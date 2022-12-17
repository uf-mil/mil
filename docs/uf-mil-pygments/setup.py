from setuptools import setup

setup(
    name="uf-mil-pygments",
    version="1",
    py_modules=["uf_mil_pygments"],
    entry_points={"pygments.styles": "mil=uf_mil_pygments:MILStyle"},
)
