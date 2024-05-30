import pyPS4Controller.controller


class PS4Controller(pyPS4Controller.controller.Controller):
    def __init__(self, **kwargs):
        pyPS4Controller.controller.Controller.__init__(self, **kwargs)
        ...
    ...