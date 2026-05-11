def make_string_list(value):
    """
    Convert a parameter or grounding field to a list of strings.

    Args:
        value: string value or list of values.

    Returns:
        [:obj:`str`]: list of strings.
    """
    if value is None:
        return []
    if isinstance(value, str):
        return [value]
    return [str(item) for item in value]
