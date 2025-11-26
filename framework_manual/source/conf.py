# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Framework Manual'
copyright = '2025, Filipe Peixoto, Marcelo Alves'
author = 'Filipe Peixoto, Marcelo Alves'
release = '1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

#add by lip
# extensions = []
extensions = ["myst_parser"]  # optional if using Markdown
# end add by lip

templates_path = ['_templates']
exclude_patterns = ["veins_setup.rst", "troubleshooting.rst", "examples.rst", "introduction.rst"]



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# add by lip
# html_theme = 'alabaster'
html_theme = 'sphinx_rtd_theme'

html_theme_options = {
    'collapse_navigation': False,
    'navigation_depth': 2,   # or higher if you have sub-sections
}
# end add by lip

html_static_path = ['_static']
