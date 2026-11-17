"""Allows `python -m Vehicle.Suspension` to start the app."""

import multiprocessing


if __name__ == "__main__":

    multiprocessing.freeze_support()

    from Vehicle.Suspension.gui import main

    main()
