def print_debug(message):
    """Prints a debug message to the terminal in cyan."""
    cyan_color = '\033[96m'
    reset_color = '\033[0m'
    print(f"{cyan_color}DEBUG: {message}{reset_color}", flush=True)

def print_info(message):
    """Prints an info message to the terminal in green."""
    green_color = '\033[92m'
    reset_color = '\033[0m'
    print(f"{green_color}INFO: {message}{reset_color}", flush=True)

def print_warning(message):
    """Prints a warning message to the terminal in yellow."""
    yellow_color = '\033[93m'
    reset_color = '\033[0m'
    print(f"{yellow_color}WARNING: {message}{reset_color}", flush=True)

def print_error(message):
    """Prints an error message to the terminal in red."""
    red_color = '\033[91m'
    reset_color = '\033[0m'
    print(f"{red_color}ERROR: {message}{reset_color}", flush=True)
