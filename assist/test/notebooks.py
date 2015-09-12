import os
import sys
from unittest import TestCase

from runipy.notebook_runner import NotebookRunner
from IPython.nbformat.current import read


class Notebooks(TestCase):
    DEVELOPMENT = 'notebooks/development.ipynb'
    TUTORIAL = 'notebooks/tutorial.ipynb'

    def setUp(self):
        raise Undead
        pass

    def tearDown(self):
        pass

    def test_development(self):
        notebook = read(open(self.NOTEBOOK), 'json')
        r = NotebookRunner(notebook, pylab=True)
        r.run_notebook()


if __name__ == '__main__':
    notebooks_test = Notebooks()
    notebooks_test.test_development()
