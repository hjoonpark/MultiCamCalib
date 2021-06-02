import logging
import sys
import os
FORMATTER = logging.Formatter("%(asctime)s — %(name)s — %(levelname)s — %(message)s")

def _get_console_handler():
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(FORMATTER)
    return console_handler

def _get_file_handler(file_path):
    file_handler = logging.FileHandler(file_path)
    file_handler.setFormatter(FORMATTER)
    return file_handler

def get_logger(logger_name, paths):
    log_dir = paths["logs"]
    os.makedirs(log_dir, exist_ok=True)

    logfile_path = os.path.join(log_dir, "{}.log".format(logger_name))
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(_get_console_handler())
    logger.addHandler(_get_file_handler(logfile_path))
    logger.propagate = False
    return logger