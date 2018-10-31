class ConstraintLevel(object):
    def __init__(self):
        self.m_ConstraintLevel = []

    def pushback(self, ConstraintBase):
        self.m_ConstraintLevel.append(ConstraintBase)


class HQPData(object):
    def __init__(self):
        self.m_HQPData = []

    def pushback(self):
        self.m_HQPData.append([])

    def data(self):
        return self.m_HQPData

    def resize(self, number):
        for i in range(0, number):
            self.pushback()

    def size(self):
        return len(self.m_HQPData)

    def setData(self, number, weight, Constraint):
        assert number < self.size()
        self.m_HQPData[number].append([weight, Constraint])
        #self.m_HQPData[number].append([weight, Constraint])
        #self.m_HQPData[number+1].append([weight, Constraint])

    def showState(self):
        print self.m_HQPData