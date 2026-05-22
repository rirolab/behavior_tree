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


# Rebuild nested parameter dictionaries from flattened ROS parameter prefixes.
def make_nested_parameter_dict(parameters):
    nested_parameters = {}
    for parameter_name, parameter in parameters.items():
        current_level = nested_parameters
        parts = str(parameter_name).split(".")
        for part in parts[:-1]:
            current_level = current_level.setdefault(part, {})
        current_level[parts[-1]] = parameter.value
    return nested_parameters
