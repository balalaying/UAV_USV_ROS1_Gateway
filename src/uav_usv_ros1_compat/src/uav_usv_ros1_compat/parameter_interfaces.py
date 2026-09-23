class SetParametersResult:
    def __init__(self, successful=False, reason=''):
        self.successful = bool(successful)
        self.reason = str(reason)

__all__ = ['SetParametersResult']
