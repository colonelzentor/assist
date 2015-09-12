#!/usr/bin/env python
from pip.req import parse_requirements
from pip.download import PipSession
from setuptools import setup, find_packages


install_reqs = parse_requirements('requirements.txt', session=PipSession())
requirements = [str(ir.req) for ir in install_reqs]

setup(name='assist',
      version='0.0.1',
      description='Aircraft Sizing, Synthesis and Integration Support Tool (ASSIST)',
      long_description=open('README.md').read(),
      download_url='https://github.com/sanbales/ASSIST',
      keywords='aircraft design sizing and synthesis',
      author='Santiago Balestrini-Robinson',
      author_email='santiago.balestrini@gtri.gatech.edu',
      url='https://github.com/sanbales/ASSIST',
      license='MIT',
      packages=find_packages(),
      install_requires=requirements,
      classifiers=['Development Status :: 3 - Alpha',
                   'License :: MIT License',
                   'Operating System :: OS Independent',
                   'Programming Language :: Python :: 2.7',
                   ],
      py_modules=['assist'])
