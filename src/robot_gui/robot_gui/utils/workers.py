from qtpy import QtCore

class _WorkerSignals(QtCore.QObject):
    done = QtCore.Signal(bool, int, str)

class CallWorker(QtCore.QRunnable):
    """Run a function in Qt threadpool and emit (ok, code, msg)."""
    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self.fn = fn
        self.args = args
        self.kwargs = kwargs
        self.signals = _WorkerSignals()

    @QtCore.Slot()
    def run(self):
        try:
            ok, code, msg = self.fn(*self.args, **self.kwargs)
        except Exception as e:
            ok, code, msg = False, 255, f"exception: {e}"
        self.signals.done.emit(bool(ok), int(code), str(msg))
