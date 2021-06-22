from helper import load_corner_txt, load_img, convert_sec
import numpy as np
import os
import glob
import cv2
import time
from tqdm import tqdm
import torch
from torch import nn
from torch.nn import functional as F
import matplotlib.pyplot as plt
import json
class View(nn.Module):
    def __init__(self, size):
        super(View, self).__init__()
        self.size = size

    def forward(self, tensor):
        return tensor.view(self.size)

class VAE_ConvConv(nn.Module):
    # https://arxiv.org/pdf/1312.6114.pdf
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.device = kwargs.get('device', None)
        self.debug = kwargs.get('debug', False)
        in_channels = kwargs.get('in_channels', 1)
        self.z_dim = kwargs.get('z_dim', 2)

        # ------------------------------- #
        # 15x15 crop
        # encoder
        self.encoder = nn.Sequential(
            nn.Conv2d(in_channels, out_channels=16,kernel_size=3, stride=2, padding=0),
            nn.ReLU(),
            nn.Conv2d(16, out_channels=32,kernel_size=3, stride=1, padding=0),
            nn.ReLU(),
            nn.Conv2d(32, out_channels=64, kernel_size=3, stride=1, padding=0),
            nn.ReLU(),
            View((-1, 64*3*3)),
            nn.Linear(64*3*3, self.z_dim*2)
        )
        # decoder
        self.decoder = nn.Sequential(
            nn.Linear(self.z_dim, 64*3*3),
            View((-1, 64, 3, 3)),
            nn.ReLU(),
            nn.ConvTranspose2d(64, out_channels=32, kernel_size=3, stride=1, padding=0),
            nn.ReLU(),
            nn.ConvTranspose2d(32, out_channels=16, kernel_size=3, stride=1, padding=0),
            nn.ReLU(),
            nn.ConvTranspose2d(16, out_channels=in_channels, kernel_size=3, stride=2, padding=0),
            nn.Sigmoid()
        )

        if self.debug:
            print("self.encoder:", self.encoder)
            print("self.decoder:", self.decoder)

    def encode(self, x):
        if self.debug:
            print(">> encode <<")
            print('  self.encoder:', self.encoder)
            print("  x:", x.size(), x.is_cuda)
        h = self.encoder(x)
        if self.debug:
            print("  h:", h.size())
        mu = h[:, :self.z_dim]
        logvar = h[:, self.z_dim:]

        if self.debug:
            print("  mu:", mu.size())
            print("  logvar:", logvar.size())
        return mu, logvar

    def decode(self, z):
        recons = self.decoder(z)
        if self.debug:
            print(">> decode <<")
            print("  z:", z.size())
            print("  recons:", recons.size())
        return recons

    def reparameterize(self, mu, logvar):
        """
        :mu: [B x D]
        :logvar: [B x D]
        :return: [B x D]
        """
        if self.debug:
            print(">> reparameterize <<")
        std = torch.exp(0.5 * logvar)
        eps = torch.empty(std.size()).normal_()

        if mu.is_cuda and not eps.is_cuda:
            eps = eps.to(self.device)
        return mu + eps*std

    def forward(self, x, is_training=False):
        mu, logvar = self.encode(x)
        if self.debug:
            print("mu:", mu.shape, ' | logvar:', logvar.shape)
        if is_training:
            z = self.reparameterize(mu, logvar)
        else:
            z = mu
        x_recon = self.decode(z)
        return x_recon, mu, logvar

#
#
#
#
#
#
#
#
#
#
def __crop_corners(img, corners, crop_size=15):
    s = int(crop_size / 2)
    crops = []
    for corner_idx in range(len(corners)):
        center = np.int32(corners[corner_idx])
        crop = img[center[1]-s:center[1]+s+1, center[0]-s:center[0]+s+1]

        if crop.shape[0] != crop_size or crop.shape[1] != crop_size:
            continue
        else:
            crops.append(crop)
        
    crops = np.array(crops)
    return crops

def __binarize(img):
    # Otsu's thresholding after Gaussian filtering
    img = cv2.GaussianBlur(img, (5, 5), 0)
    img_out = np.array(255*img).astype('uint8')
    _, img_out = cv2.threshold(img_out, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    img_out = np.float32(img_out) / 255.0
    return img_out

def generate_crops_around_corners(logger, img_paths, paths, crop_size=15):
    dir_corners = paths["corners"]
    dir_cornercrops = paths["corner_crops"]
    
    os.makedirs(dir_cornercrops, exist_ok=True)
    save_path = os.path.join(dir_cornercrops, "crop_metadata.json")

    # index image_name corner_idx_of_curr_camera corner_idx
    crop_metadata = {}
    crop_idx = 0

    for cam_idx in sorted(list(img_paths.keys())):
        crops_gray = None # (M, H, W): M images (H x W pixels)
        crops_binary = None

        pbar = tqdm(total=len(img_paths[cam_idx]))
        for img_path in img_paths[cam_idx]:
            pbar.update(1)
            fname = os.path.split(img_path)[-1].split(".")[0]
            corner_path = os.path.join(dir_corners, "cam_{}".format(cam_idx), "{}.txt".format(fname))
            corners, _ = load_corner_txt(corner_path)

            if corners is not None:
                # load original image
                img = load_img(img_path)
                
                # crop centered around corners
                crops_curr = __crop_corners(img, corners, crop_size=crop_size)
                
                # binarize
                crops_binary_curr = []
                for corner_idx in range(len(crops_curr)):
                    b = __binarize(crops_curr[corner_idx])
                    crops_binary_curr.append(b)
                crops_binary_curr = np.array(crops_binary_curr)

                if crops_gray is None:
                    n_crops = 0
                    crops_gray = crops_curr
                    crops_binary = crops_binary_curr
                else:
                    n_crops = len(crops_gray)
                    crops_gray = np.vstack([crops_gray, crops_curr])
                    crops_binary = np.vstack([crops_binary, crops_binary_curr])

                # keep indices of corners to map back to the image
                for corner_idx in range(len(crops_curr)):
                    crop_metadata[crop_idx] = {"img_name": fname, "camera_idx": cam_idx, "corner_idx": corner_idx, "idx_for_curr_cam": n_crops+corner_idx}
                    crop_idx += 1

        # save crop
        save_path1 = os.path.join(dir_cornercrops, "{}_gray.npy".format(cam_idx))
        np.save(save_path1, crops_gray)

        save_path2 = os.path.join(dir_cornercrops, "{}_binary.npy".format(cam_idx))
        np.save(save_path2, crops_binary)
        logger.info("Crops saved (camera {}): (gray) {}\t(binary) {}".format(cam_idx, save_path1, save_path2))
        
    # save metadata
    with open(save_path, "w+") as f:
        json.dump(crop_metadata, f, indent=4)
        logger.info("Crops metadata saved: {}".format(save_path))

def __load_corner_crops(input_paths):
    # load corner crops
    crops = None
    for in_path in input_paths:
        crop = torch.from_numpy(np.load(in_path).astype(np.float32))
        if crops is None:
            crops = crop
        else:
            crops = torch.vstack((crops, crop))
    return crops

def train_vae_outlier_detector(logger, input_paths, paths, vae_config):
    batch_size = vae_config["batch_size"]
    device = vae_config["device"]
    lr = vae_config["lr"]
    n_epochs = vae_config["n_epochs"]
    kl_weight = vae_config["kl_weight"]
    z_dim = vae_config["z_dim"]
    debug = vae_config["debug"]

    dir_vae_outlier_detector = paths["vae_outlier_detector"]

    # load corner crops
    crops = __load_corner_crops(input_paths)
        
    # reproducibility
    torch.manual_seed(0)
    np.random.seed(0)

    # truncate batch
    n_batch = len(crops) // batch_size
    crops_trunc = crops[0:batch_size*n_batch]
    logger.info("Train input: {} crops | batch_size={} | n_batch={} | trunc={} | unused={}".format(len(crops), batch_size, n_batch, len(crops_trunc), len(crops)-len(crops_trunc)))

    # initialize model
    model = VAE_ConvConv(device=device, z_dim=z_dim, debug=debug, in_channels=1).to(device)

    # initialize optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

    # save hyper parameters
    save_path = os.path.join(dir_vae_outlier_detector, "train_hyper_params.json")
    with open(save_path, "w+") as f:
        json.dump({"z_dim": z_dim, "kld_weight": kl_weight, "epochs": n_epochs, "n_batch": n_batch, "batch_size": batch_size, 'lr': lr},  f, indent=4)
    logger.info("Hyperparameters saved: {}".format(save_path))

    # save losses for monitoring
    losses = {"total": [], "kld": [], "recon": []}
    time_start = time.time()
    
    for e in tqdm(range(n_epochs)):
        # shuffle
        indices = np.random.shuffle(np.arange(0, len(crops_trunc)))    
        crops_trunc_shuffled = crops_trunc[indices].squeeze()

        # losses in current epoch
        losses_curr = {"total": 0.0, "kld": 0.0, "recon": 0.0}

        for batch_idx in range(n_batch):
            in_batch = crops_trunc_shuffled[batch_idx*batch_size:(batch_idx+1)*batch_size].unsqueeze(1).to(device)

            # forward
            recons, mu, logvar = model(in_batch, is_training=True)
            l_kld = -0.5 * torch.sum(1 + logvar - mu ** 2 - logvar.exp(), dim=1) # [batch_size]
            l_recons = torch.sum(torch.sum(((in_batch-recons)**2).squeeze(), dim=1), dim=1) # [batch_size]
            loss = torch.mean(l_recons + kl_weight*l_kld)
            
            l_kld = torch.mean(l_kld)
            l_recons = torch.mean(l_recons)

            losses_curr["total"] += loss.item() / n_batch
            losses_curr["kld"] += l_kld.item() / n_batch
            losses_curr["recon"] += l_recons.item() / n_batch

            # backward
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        dt_elpased = convert_sec(time.time() - time_start)

        for k, v in losses_curr.items():
            losses[k].append(v)

        if len(losses["total"]) > 0:
            tqdm.write('Epoch {}/{}\telapsed={} | loss: initial={:.4f} curr={:.4f} | recon loss={:.4f}, kld loss={:.4f}'.format(
                e+1, n_epochs, dt_elpased, losses["total"][0], losses["total"][-1], losses["recon"][-1], losses["kld"][-1])
                )
        
        dt_elpased = convert_sec(time.time() - time_start)
        
        if (e > 0 and e % 20 == 0) or e == n_epochs - 1:
            # save model
            model_save_path = os.path.join(dir_vae_outlier_detector, "vae_model.pt")
            torch.save(model, model_save_path)
            print("\nModel saved: {}".format(model_save_path))

            plt.figure()
            plt.plot(losses["total"], linewidth=3, label="total")
            plt.plot(losses["recon"], linewidth=1, label="recon")
            plt.plot(losses["kld"], linewidth=1, label="kld")

            plt.legend(loc="upper right")
            plt.title("Train loss (epoch={})\nfinal loss={:.4f} | recon={:.4f}, kld={:.4f}".format(e+1, losses["total"][-1], losses["recon"][-1], losses["kld"][-1]))
            plt.grid()
            plt.yscale("log")
            plt.xlabel("epoch")
            plt.ylabel("loss")
            plt.xlim([0, n_epochs])

            save_path = os.path.join(dir_vae_outlier_detector, "train_loss_plot.png")
            plt.savefig(save_path, dpi=150)
            print("Train plot saved: {}".format(save_path))
            plt.close()

            if e == n_epochs - 1:
                logger.info("Train plot saved: {}".format(save_path))

    model_save_path = os.path.join(dir_vae_outlier_detector, "vae_model.pt")
    torch.save(model, model_save_path)
    logger.info("Model saved: {}".format(model_save_path))

def run_vae_outlier_detector(logger, input_paths, paths, model_path, vae_config, save_result_imgs=False):
    if not save_result_imgs:
        batch_size = vae_config["batch_size"]
    else:
        batch_size = 1
    device = vae_config["device"]

    # save forward run reconstruction losses
    output_path = os.path.join(paths["vae_outlier_detector"], "vae_forward_result.json")
    result = {}

    crops = __load_corner_crops(input_paths)
    
    # for reproducibility
    torch.manual_seed(0)
    np.random.seed(0)

    # load model
    model = torch.load(model_path)
    model.eval()
    model.debug = False

    losses = {"recon": []}
    pbar = tqdm(total=len(crops))

    if save_result_imgs:
        outputs = []
    for i in range(len(crops)):
        input_crops = crops[i].unsqueeze(0).unsqueeze(1).to(device)

        # forward
        recons, _, _ = model(input_crops, is_training=False)

        # losses
        # l_kld = -0.5 * torch.sum(1 + logvar - mu ** 2 - logvar.exp())
        l_recons = float(torch.sum(((input_crops-recons)**2)).detach().squeeze().cpu())
        
        losses["recon"].append(l_recons)

        # save losses
        result[i] = l_recons

        if save_result_imgs:
            outputs.append({"idx": i, "input": input_crops.squeeze().detach().cpu().numpy(), "recon": recons.squeeze().detach().cpu().numpy(), "loss": l_recons})

        if i % batch_size == 0:
            pbar.update(batch_size)

    with open(output_path, 'w+') as f:
        json.dump(result, f, indent=4)

    logger.info("VAE forward result saved to: {}".format(output_path))

    if save_result_imgs:
        n_crops = len(outputs)
        n_cols = 10
        n_rows_max = 10

        n_rows, rem = divmod(n_crops, n_cols)
        n_plots, rem2 = divmod(n_rows*2, n_rows_max)

        if rem2 > 0:
            n_plots += 1
        if rem > 0:
            n_rows += 1
        
        save_dir = os.path.join(paths["vae_outlier_detector"], "forward_result")
        os.makedirs(save_dir, exist_ok=True)

        crop_idx = 0
        for plot_idx in range(n_plots): 
            save_path = os.path.join(save_dir, "forward_{}.png".format(plot_idx))
            s = 2
            fig, ax = plt.subplots(nrows=n_rows_max, ncols=n_cols, squeeze=True, figsize=(s*n_cols, 1.2*s*n_rows_max))
            ax = ax.ravel()
            for r in range(0, n_rows_max, 2):
                for c in range(n_cols):
                    a0 = ax[r*n_cols + c]
                    a1 = ax[(r+1)*n_cols + c]
                    if crop_idx < len(crops):
                        output = outputs[crop_idx]
                        a0.imshow(output["input"], cmap="gray")
                        a0.set_title("({}/{})\n{} | {:.2f}".format(crop_idx+1, n_crops, output["idx"], output["loss"]))
                        a0.set_xticks([])
                        a0.set_yticks([])

                        a1.imshow(output["recon"], cmap="gray")
                        a1.set_title("({}/{})\nreconstructed".format(crop_idx+1, n_crops))
                        a1.set_xticks([])
                        a1.set_yticks([])

                        crop_idx += 1
                    else:
                        a0.axis(False)
                        a1.axis(False)
            fig.subplots_adjust(top=0.95)
            plt.suptitle("[{}/{}] {}/{} forwards".format(plot_idx+1,n_plots, (plot_idx+1)*n_cols*n_rows_max, len(crops)))
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close()
            logger.info("Forward plot saved [{}/{}]: {}".format(plot_idx+1, n_plots, save_path))


def determine_outliers(logger, vae_config, model_path, input_paths, paths, save_path, outlier_thres_ratio=0.001, save_imgs=False):
    input_paths = sorted(list(glob.glob(os.path.join(paths["corner_crops"], "*_binary.npy"))))
    input_gray_paths = sorted(list(glob.glob(os.path.join(paths["corner_crops"], "*_gray.npy"))))
    
    os.makedirs(paths["outliers"], exist_ok=True)
    
    # load crops
    crops = __load_corner_crops(input_paths)
    if save_imgs:
        crops_gray = __load_corner_crops(input_gray_paths)

    # crop metadata
    with open(os.path.join(paths["corner_crops"], "crop_metadata.json"), "r") as f:
        crop_md = json.load(f)
    
    # recon loss from forward vae
    with open(os.path.join(paths["vae_outlier_detector"], "vae_forward_result.json"), "r") as f:
        vae_result = json.load(f)

    corner_indices = np.array(list(vae_result.keys()))
    recon_losses = []
    for i in corner_indices:
        recon_losses.append(vae_result[i])
    recon_losses = np.array(recon_losses)

    idx = np.argsort(recon_losses)[::-1]
    sorted_losses = recon_losses[idx]
    sorted_indices = corner_indices[idx]

    n_items = len(corner_indices)
    outliers_indices = sorted_indices[0:int(n_items*outlier_thres_ratio)]
    outliers_losses = sorted_losses[0:int(n_items*outlier_thres_ratio)]

    outliers = {}
    for i in range(len(outliers_indices)):
        crop_idx = outliers_indices[i]
        loss = outliers_losses[i]
        md = crop_md[str(crop_idx)]
        outliers[crop_idx] = {"crop_idx": int(crop_idx), "recon_loss": loss, "img_name": md["img_name"], "camera_idx": md["camera_idx"], "corner_idx": md["corner_idx"], "idx_for_curr_cam": md["idx_for_curr_cam"]}

    with open(save_path, "w+") as f:
        json.dump(outliers, f, indent=4)

    logger.info("Outliers saved to: {}".format(save_path))

    if save_imgs:
        # load vae model to run forward passes
        device = vae_config["device"]

        # load model
        model = torch.load(model_path)
        model.eval()
        model.debug = False

        save_dir = paths["outliers"]
        os.makedirs(save_dir, exist_ok=True)

        # load corner images
        mds = []
        crops_for_plot = []
        crops_recon = []
        pbar = tqdm(total=len(outliers.keys()))
        for _, md in outliers.items():
            pbar.update(1)
            
            crop_idx = int(md["crop_idx"])
            crop_binary = crops[crop_idx, :, :]
            crop_gray = crops_gray[crop_idx, :, :]

            mds.append(md)

            # forward
            input_crop = crop_binary.unsqueeze(0).unsqueeze(1).to(device)
            recon, _, _ = model(input_crop, is_training=False)
            recon = recon.detach().squeeze().cpu().numpy()

            crops_recon.append(recon)
            crops_for_plot.append(crop_gray.numpy())

        n_crops = len(crops_for_plot)
        n_cols = 10
        n_rows_max = 10

        n_rows, rem = divmod(n_crops, n_cols)
        n_plots, rem2 = divmod(n_rows*2, n_rows_max)

        if rem2 > 0:
            n_plots += 1
        if rem > 0:
            n_rows += 1
        
        i = 0
        for plot_idx in range(n_plots): 
            save_path = os.path.join(save_dir, "outliers_{}.png".format(plot_idx))
            s = 2
            fig, ax = plt.subplots(nrows=n_rows_max, ncols=n_cols, squeeze=True, figsize=(s*n_cols, 1.2*s*n_rows_max))
            ax = ax.ravel()
            for r in range(0, n_rows_max, 2):
                for c in range(n_cols):
                    a0 = ax[r*n_cols + c]
                    a1 = ax[(r+1)*n_cols + c]
                    if i < len(crops_for_plot):
                        md = mds[i]
                        a0.imshow(crops_for_plot[i], cmap="gray")
                        a0.set_title("({}/{})\n{} | {:.2f}".format(i+1, n_crops, md["crop_idx"], md["recon_loss"]))
                        a0.set_xlabel(md["img_name"])

                        a0.set_xticks([])
                        a0.set_yticks([])

                        a1.imshow(crops_recon[i], cmap="gray")
                        a1.set_title("({}/{})\nreconstructed".format(i+1, n_crops))
                        a1.set_xticks([])
                        a1.set_yticks([])

                        i += 1
                    else:
                        a0.axis(False)
                        a1.axis(False)
            fig.subplots_adjust(top=0.95)
            plt.suptitle("[{}/{}] {}/{} outlier corners (total={})".format(plot_idx+1,n_plots, (plot_idx+1)*n_cols*n_rows_max, len(crops_for_plot), n_items))
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            plt.close()
            logger.info("Outlier plot saved [{}/{}]: {}".format(plot_idx+1, n_plots, save_path))