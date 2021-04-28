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
        # deencoder
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












def __crop_corners(img, corners, crop_size=15):
    s = int(crop_size / 2)
    crops = []
    for corner_idx in range(len(corners)):
        center = np.int32(corners[corner_idx])
        crop = img[center[1]-s:center[1]+s+1, center[0]-s:center[0]+s+1]
        crops.append(crop)
    return np.array(crops)

def __binarize(img):
    # Otsu's thresholding after Gaussian filtering
    img = cv2.GaussianBlur(img, (5, 5), 0)
    img_out = np.array(255*img).astype('uint8')
    _, img_out = cv2.threshold(img_out, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    img_out = np.float32(img_out) / 255.0
    return img_out

def generate_crops_around_corners(img_paths, output_dir, crop_size=15):
    print(" * Generate crops around corners")
    save_folder = os.path.join(output_dir, "corner_crops")
    os.makedirs(save_folder, exist_ok=True)

    crop_metadata = []
    crop_metadata.append("# index image_name corner_idx_of_curr_camera corner_idx\n")

    for cam_idx in sorted(list(img_paths.keys())):
        crops_gray = None # (M, H, W): M images (H x W pixels)
        crops_binary = None
        for img_path in img_paths[cam_idx]:
            fname = os.path.split(img_path)[-1].split(".")[0]
            corner_path = os.path.join(output_dir, "corners", "cam_{}".format(cam_idx), "{}.txt".format(fname))
            corners = load_corner_txt(corner_path)

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
                    crop_metadata.append("{} {} {} {}\n".format(len(crop_metadata), fname, n_crops+corner_idx, corner_idx))

        # save crop
        save_path1 = os.path.join(save_folder, "{}_gray.npy".format(cam_idx))
        np.save(save_path1, crops_gray)

        save_path2 = os.path.join(save_folder, "{}_binary.npy".format(cam_idx))
        np.save(save_path2, crops_binary)
        print("  - crop saved: (gray) {}\t(binary) {}".format(save_path1, save_path2))
        
    # save metadata
    save_path = os.path.join(save_folder, "crop_metadata.txt")
    with open(save_path, "w+") as f:
        f.writelines(crop_metadata)
        print("  - metadata saved: {}".format(save_path))

def __load_corner_crops(input_paths):
    # load corner crops
    crops = None
    for in_path in input_paths:
        crop = torch.from_numpy(np.load(in_path).astype(np.float32))
        if crops is None:
            crops = crop
        else:
            crops = torch.vstack([crops, crop])
    return crops

def train_vae_outlier_detector(input_paths, output_dir, vae_configs):
    batch_size = vae_configs["batch_size"]
    device = vae_configs["device"]
    lr = vae_configs["lr"]
    n_epochs = vae_configs["n_epochs"]
    kl_weight = vae_configs["kl_weight"]
    z_dim = vae_configs["z_dim"]
    debug = vae_configs["debug"]

    save_dir = os.path.join(output_dir, "vae_outlier_detector")
    os.makedirs(save_dir, exist_ok=True)

    # load corner crops
    crops = __load_corner_crops(input_paths)
        
    # reproducibility
    torch.manual_seed(0)
    np.random.seed(0)

    # truncate batch
    n_batch = len(crops) // batch_size
    crops_trunc = crops[0:batch_size*n_batch]
    print("  - INPUT: {} crops | batch_size={} | n_batch={} | trunc={} | unused={}".format(len(crops), batch_size, n_batch, len(crops_trunc), len(crops)-len(crops_trunc)))

    # shuffle manually
    indices = np.random.shuffle(np.arange(0, len(crops_trunc)))    
    crops_trunc_shuffled = crops_trunc[indices].squeeze()

    # initialize model
    model = VAE_ConvConv(device=device, z_dim=z_dim, debug=debug, in_channels=1).to(device)

    # initialize optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)

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
            t0 = time.time()
            in_batch = crops_trunc[batch_idx*batch_size:(batch_idx+1)*batch_size].unsqueeze(1).to(device)
            targets = crops_trunc[batch_idx*batch_size:(batch_idx+1)*batch_size].unsqueeze(1).to(device)

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
            tqdm.write('[{}/{}] e[{}/{}]\telapsed={} | loss: initial={:.4f} curr epoch={:.4f} | recon loss={:.4f}, kld loss={:.4f}'.format(
                e*n_batch+batch_idx, n_epochs*n_batch-1, e, n_epochs-1, dt_elpased, losses["total"][0], losses["total"][-1], losses["recon"][-1], losses["kld"][-1])
                )
        
        dt_elpased = convert_sec(time.time() - time_start)
        
        if (e > 0 and e % 20 == 0) or e == n_epochs - 1:
            # save model
            model_save_path = os.path.join(save_dir, "vae_model.pt")
            torch.save(model, model_save_path)
            print("  - Model saved: {}".format(model_save_path))

            plt.figure()
            plt.plot(losses["total"], linewidth=3, label="total")
            plt.plot(losses["recon"], label="recon")
            plt.plot(losses["kld"], label="kld")

            plt.legend(loc="upper right")
            plt.title("Train loss (epoch={})\nfinal loss={:.4f} | recon={:.4f}, kld={:.4f}".format(e+1, losses["total"][-1], losses["recon"][-1], losses["kld"][-1]))
            plt.grid()
            plt.yscale("log")
            plt.xlabel("epoch")
            plt.ylabel("loss")

            save_path = os.path.join(save_dir, "train_loss_plot.png")
            plt.savefig(save_path, dpi=150)
            print("  - Plot saved: {}".format(save_path))

    model_save_path = os.path.join(save_dir, "vae_model.pt")
    torch.save(model, model_save_path)
    print("  - Model saved: {}".format(model_save_path))

    save_path = os.path.join(save_dir, "train_hyper_params.json")
    with open(save_path, "w+") as f:
        json.dump({"z_dim": z_dim, "kld_weight": kl_weight, "epochs": n_epochs, "n_batch": n_batch, "batch_size": batch_size, 'lr': lr},  f, indent=4)

    print("  - Hyper parameters saved: {}".format(save_path))

def run_vae_outlier_detector(input_paths, output_dir, model_path, vae_configs):
    batch_size = vae_configs["batch_size"]
    device = vae_configs["device"]

    crops = __load_corner_crops(input_paths)
    
    # for reproducibility
    torch.manual_seed(0)
    np.random.seed(0)

    # load model
    model = torch.load(model_path)
    model.eval()
    model.debug = False

    time_start = time.time()
    losses = {"recon": []}
    for i in tqdm(range(len(crops))):
        input_crops = crops[i].unsqueeze(0).unsqueeze(1).to(device)

        # forward
        recons, mu, logvar = model(input_crops, is_training=False)

        # losses
        l_kld = -0.5 * torch.sum(1 + logvar - mu ** 2 - logvar.exp())
        l_recons = torch.sum(((input_crops-recons)**2))

        losses["recon"].append(l_recons.detach().squeeze().cpu().numpy())

        if i % 100 == 0:
            tqdm.write("[{}/{}]\telapsed={}".format(i, len(crops-1), convert_sec(time.time()-time_start)))
