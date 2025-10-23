# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'pyenki'
copyright = '2020, Stéphane Magnenat and others'
author = 'Stéphane Magnenat and others. This version is maintained by Jerome Guzzi.'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.autosectionlabel',
]

highlight_language = 'python'

add_module_names = False
autodoc_typehints_format = 'short'
autodoc_member_order = 'groupwise'
autodoc_class_signature = 'mixed'
autodoc_inherit_docstrings = True
autoclass_content = 'class'
autodoc_docstring_signature = True

# autodoc_typehints_format = 'short'
# autodoc_member_order = 'groupwise'
# autodoc_class_signature = 'mixed'
# autodoc_inherit_docstrings = True
# autoclass_content = 'class'
# autodoc_docstring_signature = True
# autodoc_typehints = 'signature'

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# html_theme = 'nature'
html_theme = 'sphinx_book_theme'
html_static_path = ['_static']
