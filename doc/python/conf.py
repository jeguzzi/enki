# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

from sphinx.addnodes import pending_xref

project = 'pyenki'
copyright = '2020, Stéphane Magnenat and others'
author = 'Stéphane Magnenat and others. This version is maintained by Jerome Guzzi.'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc', 'sphinx.ext.napoleon', 'sphinx.ext.autosectionlabel',
    'sphinx_toolbox.code', 'sphinx_tabs.tabs', 'sphinx.ext.intersphinx',
    'enum_tools.autoenum', 'sphinxcontrib.video'
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable', None),
}

autodoc_default_options = {
    'members': True,
    'special-members': False,
    'private-members': False,
    'inherited-members': False,
    'undoc-members': False,
    'exclude-members': '__weakref__',
}

autodoc_type_aliases = {
    'Vector': 'Vector',
    'Vector3': 'Vector3',
    'VectorLike': 'VectorLike',
    'Vector3Like': 'Vector3Like',
    'Image': 'Image',
    'Array1D': 'Array1D',
    'Array2D': 'Array2D',
    'ARGBImage': 'ARGBImage',
    'ARGBImageLike': 'ARGBImageLike',
    'IntArray1D': 'IntArray1D'
}

highlight_language = 'python'

# add_module_names = False
autodoc_typehints_format = 'short'
autodoc_member_order = 'groupwise'
autodoc_class_signature = 'separated'
# autodoc_inherit_docstrings = True
autoclass_content = 'class'
autodoc_docstring_signature = True

autodoc_default_options = {
    # 'show-inheritance': False,
    'exclude-members': '__new__'
}

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

_replace = {
    "pyenki.pyenki": "pyenki",
    # "pyenki.": "",
    "collections.abc.": "",
    "typing.": ""
}

_types = [
    'Vector',
    'VectorLike',
    'Vector3',
    'Vector3Like',
    'Image',
    'ARGBImage',
    'ARGBImageLike',
    'Array1D',
    'Array2D',
    'IntArray1D'
]

_attrs = [
    'numpy.uint8', 'numpy.float64', 'numpy.float32', 'numpy.int64',
    'numpy.int32'
]

_data = ['numpy.typing.NDArray', 'numpy.typing.ArrayLike']

_meths = []

aliases = {
    "Sequence": "collections.abc.Sequence",
    "Callable": "collections.abc.Callable",
    "SupportsFloat": "typing.SupportsFloat",
    "SupportsInt": "typing.SupportsInt",
    "Unpack": "typing.Unpack",
    "Annotated": "typing.Annotated",
    "Any": "typing.Any",
}


def f_docstring(app, what, name, obj, options, lines):
    for i, _ in enumerate(lines):
        if 'self' in lines[i]:
            import re

            lines[i] = re.sub(r"self: (\w+\.?)+,?\s*", "", lines[i])
        for k, v in _replace.items():
            if k in lines[i]:
                lines[i] = lines[i].replace(k, v)


def f_signature(app, what, name, obj, options, signature, return_annotation):
    if signature:
        import re

        signature = re.sub(r"self: (\w+\.?)+,?\s*", "", signature)
        for k, v in _replace.items():
            if k in signature:
                signature = signature.replace(k, v)
    if return_annotation:
        for k, v in _replace.items():
            if k in return_annotation:
                return_annotation = return_annotation.replace(k, v)
    return (signature, return_annotation)


def resolve_internal_aliases(app, doctree):
    pending_xrefs = doctree.traverse(condition=pending_xref)
    for node in pending_xrefs:
        if node['refdomain'] == "py":
            if node['reftarget'] in _types:
                node["reftype"] = "type"
            elif node['reftarget'] in _attrs:
                node["reftype"] = "attr"
            elif node['reftarget'] in _data:
                node["reftype"] = "data"
            elif node['reftarget'] in _meths:
                node["reftype"] = "meth"
        alias = node.get('reftarget', None)
        if alias is not None and alias in aliases:
            node['reftarget'] = aliases[alias]


def setup(app):
    app.connect('doctree-read', resolve_internal_aliases)
    app.connect('autodoc-process-docstring', f_docstring)
    app.connect('autodoc-process-signature', f_signature)
