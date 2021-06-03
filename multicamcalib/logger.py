import logging
import sys
import os
FORMATTER = logging.Formatter("%(asctime)s | %(name)s | %(levelname)s | %(message)s")

def init_logger(logger_name, log_dir, init_console_handler=True, init_file_handler=True):
    os.makedirs(log_dir, exist_ok=True)

    logfile_path = os.path.join(log_dir, "{}.log".format(logger_name))
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.DEBUG)

    if init_file_handler:
        fh = logging.FileHandler(logfile_path)
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(FORMATTER)
        logger.addHandler(fh)

    if init_console_handler:
        sh = logging.StreamHandler()
        sh.setLevel(logging.INFO)
        sh.setFormatter(FORMATTER)
        logger.addHandler(sh)

    logger.propagate = False
    return logger