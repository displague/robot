"""mkdocs hook (mkdocs.yml `hooks:`): regenerate docs/index.md from the docs' front matter before each build."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import docs_index  # noqa: E402


def on_pre_build(config, **kwargs):
    docs_index.write(config["docs_dir"])
