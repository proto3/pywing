from PyQt5 import QtCore, QtGui, Qt

class CutParametersModel(QtCore.QObject):
    update = QtCore.pyqtSignal()
    reset  = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.lead = 10.0
        self.feedrate = 200.0

    def export_tuple(self):
        return (self.lead, self.feedrate)

    def import_tuple(self, tuple):
        self.lead, self.feedrate = tuple
        self.reset.emit()

    def set_lead(self, l):
        self.lead = l
        self.update.emit()

    def set_feedrate(self, f):
        self.feedrate = f
        # feedrate change doesn't need to emit signal

class CutParametersWidget(QtGui.QWidget):
    def __init__(self, cut_param_model):
        super().__init__()
        self.cut_param = cut_param_model

        self.lead_spbox = QtGui.QDoubleSpinBox()
        self.lead_spbox.setRange(1, 1000)
        self.lead_spbox.setValue(self.cut_param.lead)
        self.lead_spbox.setPrefix("Lead in/out : ")
        self.lead_spbox.setSuffix("mm")
        self.lead_spbox.valueChanged.connect(self.on_lead_change)

        self.feedrate_spbox = QtGui.QDoubleSpinBox()
        self.feedrate_spbox.setRange(1, 1000)
        self.feedrate_spbox.setValue(self.cut_param.feedrate)
        self.feedrate_spbox.setSingleStep(10)
        self.feedrate_spbox.setPrefix("Feedrate : ")
        self.feedrate_spbox.setSuffix("mm/s")
        self.feedrate_spbox.valueChanged.connect(self.on_feedrate_change)

        self.widgets = (self.lead_spbox, self.feedrate_spbox)

        layout = QtGui.QVBoxLayout()
        [layout.addWidget(w) for w in self.widgets]
        self.setLayout(layout)

        self.cut_param.reset.connect(self.reset)

    def on_lead_change(self):
        self.cut_param.set_lead(self.lead_spbox.value())

    def on_feedrate_change(self):
        self.cut_param.set_feedrate(self.feedrate_spbox.value())

    def reset(self):
        [w.blockSignals(True) for w in self.widgets]
        self.lead_spbox.setValue(self.cut_param.lead)
        self.feedrate_spbox.setValue(self.cut_param.feedrate)
        [w.blockSignals(False) for w in self.widgets]
