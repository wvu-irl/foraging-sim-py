import config

def debugPrint(*args):
    if config.enable_debug_prints:
        print(*args)
    else:
        pass
