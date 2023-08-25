#!/usr/bin/env python3
import stretch_body
from pathlib import Path
import shutil

from pdoc import pdoc, doc
from pdoc import render

# pdoc -t ./custom-template/ stretch_body -o . --logo ../images/banner.png
if __name__ == "__main__":
    doc = doc.Module(stretch_body)
    
    # Configure Pdoc render
    # https://pdoc.dev/docs/pdoc/render.html#configure
    render.configure(logo="../images/banner.png",
                     logo_link='https://github.com/hello-robot/stretch_body',
                     template_directory="./custom-template/",
                     favicon="../images/hello_robot_favicon.png",
                     edit_url_map="stretch_body=https://github.com/hello-robot/stretch_body/tree/feature/pdoc/body/stretch_body/")

    out = render.html_module(module=doc, all_modules={"stretch_body": doc})

    with open("stretch_body.html", "w") as f:
        f.write(out)

# MKdocs support: https://github.com/mitmproxy/pdoc/tree/main/examples/mkdocs