#!/usr/bin/python3
import re

import numpy as np
import urllib2

"""
Generate a random name from a wikipedia article.
Before you freak out - I want to do this so I dont overwrite segmented images.
I want to arbitrarily add and remove images without having to scan the folder every time.
Plus this is more entertaining.
"""


def name_gen(no_web=False):
    while True:
        try:
            if no_web:
                raise Exception

            url = "http://en.wikipedia.org/wiki/Special:Random"
            req = urllib2.Request(url, None, {"User-Agent": "x"})
            page = urllib2.urlopen(req).readlines()

            title = None
            for line in page:
                if "<title>" in line:
                    title = line
                    break

            if title is None:
                raise Exception

            """
            Ex of what will be in page:
              <title>List of United States Supreme Court cases, volume 457 - Wikipedia, the free encyclopedia</title>\n
            We just want:
              list_of_united_states_supreme_court_cases_volume_457
            """
            stripped_title = (
                title.split("title>")[1]
                .split(" - Wikipedia, the free encyclopedia")[0]
                .replace(" ", "_")
                .lower()
            )
            stripped_title.replace("-", "_")
            # Replace any non alpha numeric characters
            stripped_title = re.sub(r"\W+", "", stripped_title)

        except Exception:
            stripped_title = np.random.randint(999999999)

        yield stripped_title
