from PyQt5 import QtCore, QtGui

class FoamBlockModel(QtCore.QObject):
    update = QtCore.pyqtSignal()
    reset  = QtCore.pyqtSignal()

    def __init__(self, machine):
        super().__init__()
        self.machine = machine
        self.width = int(self.machine.get_width() / 2)
        self.offset = int(self.machine.get_width() / 4)
        self.machine.properties_changed.connect(self.on_machine_change)

    def export_tuple(self):
        return (self.width, self.offset)

    def import_tuple(self, tuple):
        self.width, self.offset = tuple
        self.reset.emit()

    def reverse(self):
        self.offset = self.machine.get_width() - self.width - self.offset
        self.reset.emit()
        self.update.emit()

    def set_width(self, w):
        self.width = w
        self.offset = min(self.offset, self.get_max_offset())
        self.reset.emit()
        self.update.emit()

    def set_offset(self, o):
        self.offset = o
        self.update.emit()

    def get_max_width(self):
        return self.machine.get_width()

    def get_max_offset(self):
        return self.machine.get_width() - self.width

    def on_machine_change(self):
        self.width = min(self.width, self.get_max_width())
        self.offset = min(self.offset, self.get_max_offset())
        self.reset.emit()
        self.update.emit()

class FoamBlockWidget(QtGui.QWidget):
    def __init__(self, foam_block_model):
        super().__init__()
        self.block = foam_block_model

        self.width_spbox = QtGui.QDoubleSpinBox()
        self.width_spbox.setRange(1, self.block.get_max_width())
        self.width_spbox.setValue(self.block.width)
        self.width_spbox.setPrefix("Block width : ")
        self.width_spbox.setSuffix("mm")
        self.width_spbox.valueChanged.connect(self.on_width_change)

        self.offset_spbox = QtGui.QDoubleSpinBox()
        self.offset_spbox.setRange(0, self.block.get_max_offset())
        self.offset_spbox.setValue(self.block.offset)
        self.offset_spbox.setPrefix("Block offset : ")
        self.offset_spbox.setSuffix("mm")
        self.offset_spbox.valueChanged.connect(self.on_offset_change)

        self.widgets = (self.width_spbox, self.offset_spbox)

        layout = QtGui.QVBoxLayout()
        [layout.addWidget(w) for w in self.widgets]
        self.setLayout(layout)

        self.block.reset.connect(self.reset)

    def on_width_change(self):
        self.block.set_width(self.width_spbox.value())

    def on_offset_change(self):
        self.block.set_offset(self.offset_spbox.value())

    def reset(self):
        [w.blockSignals(True) for w in self.widgets]
        self.width_spbox.setValue(self.block.width)
        self.offset_spbox.setValue(self.block.offset)
        self.width_spbox.setMaximum(self.block.get_max_width())
        self.offset_spbox.setMaximum(self.block.get_max_offset())
        [w.blockSignals(False) for w in self.widgets]
