#!/usr/bin/env python
from pip.req import parse_requirements
from pip.download import PipSession
from setuptools import setup, find_packages


install_reqs = parse_requirements('requirements.txt', session=PipSession())
requirements = [str(ir.req) for ir in install_reqs]

setup(name='assist',
      version='0.0.3',
      description='An early-stage conceptual design tool for fixed-wing aircraft',
      long_description=open('README.md').read(),
      download_url='https://github.com/sanbales/assist',
      keywords='aircraft design sizing and synthesis',
      author='Santiago Balestrini-Robinson',
      author_email='sanbales@gmail.com',
      url='https://github.com/sanbales/ASSIST',
      license='MIT',
      packages=find_packages(),
      install_requires=requirements,
      classifiers=['Development Status :: 2 - Pre-Alpha',
                   'License :: OSI Approved :: MIT License',
                   'Natural Language :: English',
                   'Topic :: Scientific/Engineering',
                   'Framework :: IPython',
                   'Operating System :: MacOS :: MacOS X',
                   'Operating System :: POSIX :: Linux',
                   'Operating System :: Microsoft :: Windows',
                   'Topic :: Scientific/Engineering',
                   'Programming Language :: Python :: 2.7',
                   ],
      py_modules=['assist'])
