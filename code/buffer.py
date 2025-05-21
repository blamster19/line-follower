class CyclicBuffer:
    def __init__(self, size):
        self.size = size
        self.buffer = [0.0] * size
        self.index = 0
        self.count = 0
        self.sum = 0.0

    def append(self, value):
        # Remove the value being overwritten from the sum
        overwritten = self.buffer[self.index]
        self.sum -= overwritten
        # Add the new value
        self.buffer[self.index] = value
        self.sum += value
        # Move index
        self.index = (self.index + 1) % self.size
        # Update count (for first 'size' elements)
        if self.count < self.size:
            self.count += 1

    def average(self):
        if self.count == 0:
            return 0.0
        return self.sum / self.count

    def __len__(self):
        return self.count