import os
import sys
from unittest import TestCase

from runipy.notebook_runner import NotebookRunner
from IPython.nbformat.current import read


class Notebooks(TestCase):
    self.DEVELOPMENT = 'notebooks/development.ipynb'
    self.TUTORIAL = 'notebooks/tutorial.ipynb'
    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_development(self):
        notebook = read(open(self.NOTEBOOK), 'json')
        r = NotebookRunner(notebook, pylab=True)
        r.run_notebook()
