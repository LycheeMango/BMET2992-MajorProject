class Patient:
    def __init__(self, id, name, age, csvfilepath, logfilepath, reportpath):
        # basic info of the patient
        self.id = id
        self.name = name
        self.age = age

        # parameters generated by PPG
        self.latest_pulse_rate
        self.avg_pulse_rate
        self.latest_RR
        self.avg_RR
        self.pulse_rate_FFT
        self.warning_msg

        # file paths
        self.csvfilepath = csvfilepath
        self.logfilepath = logfilepath
        self.reportpath = reportpath
    