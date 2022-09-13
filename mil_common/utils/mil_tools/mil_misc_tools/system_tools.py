import re
import unicodedata


def slugify(value: str) -> str:
    """
    Sanitizes any string into a string that is a valid filename or URL.

    Normalizes string, converts to lowercase, removes non-alpha characters,
    and converts spaces to hyphens.

    Args:
        value (str): The value to sanitize.

    Returns:
        str: The sanitized string.
    """
    # From: https://stackoverflow.com/questions/295135/turn-a-string-into-a-valid-filename
    value = unicodedata.normalize("NFKD", value).encode("ascii", "ignore")
    value = str(re.sub(r"[^\w\s-]", r"_", value.decode("ascii")).strip().lower())
    value = str(re.sub(r"[-\s]+", "-", value))
    return value
