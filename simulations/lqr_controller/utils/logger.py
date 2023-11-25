class Logger:
  
    def __init__(self, file_name):
        self.file_name = file_name

        f = open(self.file_name,"w+")
        f.close()

        self.result = ""

    def add(self, s):
        self.result+= s

        if len(self.result) >= 65536:
            self.flush() 

    def flush(self):
        f = open(self.file_name, "a+")
        f.write(self.result)
        f.close()

        self.result = ""