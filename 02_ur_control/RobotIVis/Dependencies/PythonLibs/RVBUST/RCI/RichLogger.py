#!/usr/bin/python3
# -*- coding: utf-8 -*-
# Copyright (c) RVBUST, Inc - All rights reserved.

import datetime
import logging
import os
import time


class RichLogger(logging.Logger):
    """A text-colored Logger in python
    """
    def __init__(self, name: str, console_log_level: str = "INFO", file_log_level: str = "DEBUG", fmt: str = 'Simple', out_log_path: str = None):
        """Create a new RichLogger instance

        Args:
            name (str): logger name
            console_log_level (str, optional): log level for console. Defaults to "INFO".
            file_log_level (str, optional): log level for file. Defaults to "DEBUG".
            fmt (str, optional): 'Verbose' or 'Simple'. Defaults to 'Simple'.
            out_log_path (str, optional): file path for file handler . Defaults to "~/.Logger/log_{current_time}.log"
        """
        super().__init__(name, 'DEBUG')
        LOGGER_FILE_PATH = "{}/.Logger/log_{}.log".format(
            os.getenv("HOME"), datetime.datetime.now().strftime("%Y-%m-%d__%H_%M_%S"))
        LOGGER_FORMATTER_VERBOSE = "%(asctime)s %(name)s %(pathname)s:%(lineno)d:%(funcName)s - %(levelname)s: %(message)s"
        LOGGER_FORMATTER_SIMPLE = "%(asctime)s %(name)s %(filename)s:%(lineno)d %(levelname)s: %(message)s"
        logger = self

        # stream handler
        LOGGER_FORMATTER = LOGGER_FORMATTER_VERBOSE if fmt == "Verbose" else LOGGER_FORMATTER_SIMPLE

        try:
            import coloredlogs
            coloredlogs.install(logger=logger,
                                level=console_log_level,
                                fmt=LOGGER_FORMATTER,
                                datefmt='%Y-%m-%d %H:%M:%S',
                                milliseconds=False, reconfigure=True)
        except ImportError:
            logging.warning("coloredlogs is not installed")
            stream_handler = logging.StreamHandler()
            stream_handler.setLevel(console_log_level)
            stream_handler.setFormatter(logging.Formatter(
                LOGGER_FORMATTER, datefmt='%Y-%m-%d %H:%M:%S'))
            logger.addHandler(stream_handler)

        # file handler
        if out_log_path is None:
            out_log_path = LOGGER_FILE_PATH
        os.makedirs(os.path.dirname(out_log_path), exist_ok=True)
        file_handler = logging.FileHandler(
            out_log_path, mode='a')
        file_handler.setLevel(file_log_level)
        file_handler.setFormatter(logging.Formatter(
            LOGGER_FORMATTER, datefmt='%Y-%m-%d %H:%M:%S'))
        logger.addHandler(file_handler)

    def setLevelForConsole(self, level: str = 'INFO'):
        self.handlers[0].setLevel(level)

    def setLevelForFile(self, level: str = 'DEBUG'):
        self.handlers[1].setLevel(level)


if __name__ == "__main__":
    def Test():
        logger1 = RichLogger(
            'VerboseLogger', console_log_level='INFO', file_log_level='DEBUG', fmt='Verbose', out_log_path='./test.log')
        logger2 = RichLogger(
            'SimpleLogger', console_log_level='INFO', file_log_level='DEBUG', fmt='Simple', out_log_path='./test.log')
        logger1.debug('There is a monkey.')
        logger2.debug('There is a monkey.')
        logger1.info('There is human.')
        logger2.info('There is human.')
        logger1.debug("It's bad!")
        logger2.debug("It's bad!")
        logger1.warning('Human kills.')
        logger2.warning('Human kills.')
        logger1.error('Human beings are equal?')
        logger2.error('Human beings are equal?')
        logger1.fatal("It is the end of human world!")
        logger2.fatal("It is the end of human world!")
    Test()
