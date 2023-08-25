#!/usr/bin/env python3
import stretch_body
import path
import pdoc

# pdoc -t ./custom-template/ stretch_body -o . --logo ../images/banner.png
if __name__ == "__main__":
    doc = pdoc.doc.Module(stretch_body)
    # https://pdoc.dev/docs/pdoc/render.html#configure
    pdoc.render.configure(
	docformat: Literal['markdown', 'google', 'numpy', 'restructuredtext'] = 'restructuredtext',
	include_undocumented: bool = True,
	edit_url_map: Optional[Mapping[str, str]] = None,
	favicon: str | None = None,
	footer_text: str = '',
	logo: str | None = None,
	logo_link: str | None = None,
	math: bool = False,
	mermaid: bool = False,
	search: bool = True,
	show_source: bool = True,
	template_directory: pathlib.Path | None = None)

    # We can override most pdoc doc attributes by just assigning to them.
    # doc.get("Foo.A").docstring = "I'm a docstring for Foo.A."

    out = pdoc.render.html_module(module=doc, all_modules={"stretch_body": doc})

    with open("stretch_body.html", "w") as f:
        f.write(out)