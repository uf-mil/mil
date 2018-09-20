import unicodedata
import re


def slugify(value):
    """
    Sanitizes any string into a string that is a valid filename or url.

    Normalizes string, converts to lowercase, removes non-alpha characters,
    and converts spaces to hyphens.
    From: https://stackoverflow.com/questions/295135/turn-a-string-into-a-valid-filename
    """
    value = unicodedata.normalize('NFKD', value).encode('ascii', 'ignore')
    value = unicode(re.sub('[^\w\s-]', '_', value).strip().lower())
    value = unicode(re.sub('[-\s]+', '-', value))
    return value
