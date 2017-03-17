class MissingPerceptionObject(Exception):

    def __init__(self, missing_object, message="An object from the database is missing"):

        # Call the base class constructor with the parameters it needs
        super(MissingPerceptionObject, self).__init__(message)

        # Now for your custom code...
        self.missing_object = missing_object
        self.message = message