# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
import pathlib
from setuptools import setup

HERE = pathlib.Path(__file__).parent

README = (HERE / "README.md").read_text()

setup(
  name='pybluemo',
  packages=['pybluemo'],
  version='0.12',
  description='Interface library for the Bluemo Bluetooth motion controller.',
  long_description=README,
  url='https://github.com/mjbrown/pybluemo',
  author="Michael Brown",
  author_email="mjbrown.droid@gmail.com",
  install_requires=open('requirements.txt').read().split(),
  include_package_data=True,
  classifiers=[
      "Development Status :: 3 - Alpha",
      "Intended Audience :: Developers",
      "License :: OSI Approved :: Mozilla Public License 2.0 (MPL 2.0)",
      "Programming Language :: Python :: 3",
      "Programming Language :: Python :: 3.8",
      "Topic :: Software Development :: Libraries",
      "Operating System :: OS Independent",
  ],
)
