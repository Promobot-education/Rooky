#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re

try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup


setup(
    name="Rooky Python module",
    version="0.1.0",
    license="Apache license 2.0",
    author="Promobot",
    url="promo-bot.ru",
    project_urls={
        "Documentation": "https://promo-bot.ru",
        "Source Code": "https://github.com/Promobot-education",
    },
    description="Python modules to communicate with Rooky",
    python_requires=">=2.7",
    packages=['modbus_tk'],
    py_modules=["Rooky", "Rooky2", "Servo_ppm", "Servo", "modbus_io", "bus_handler"],
)
