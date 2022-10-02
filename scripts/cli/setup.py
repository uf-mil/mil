from setuptools import setup

setup(
    name="mil",
    version="0.0.0",
    py_modules=["mil"],
    install_requires=[
        "rich-click",
    ],
    entry_points={
        "console_scripts": [
            "mil = mil:mil",
        ],
    },
)
