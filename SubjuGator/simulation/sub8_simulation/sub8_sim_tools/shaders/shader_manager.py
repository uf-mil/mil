from shaders import Shaders


class LightManager(object):

    def __init__(self):
        self.num_lights = 0
        self.light_sensitive_entities = []
        self.lights = []
        self.base_vert = Shaders.multiple_shaders['phong']['vertex']
        self.base_frag = Shaders.multiple_shaders['phong']['fragment']

    def edit_shader(self, entity):
        new_value = "uniform vec3 u_lights[" + str(self.num_lights * 2) + "];"
        new_frag = self.base_frag.replace("//INSERT LIGHTS HERE", new_value)
        entity.program.set_shaders(self.base_vert, new_frag)

        # update the shaders values
        entity.program['u_numLights'] = self.num_lights
        for i in range(0, (len(self.lights))):
            number = "u_lights[" + str(i) + "]"
            entity.program[number] = self.lights[i]

    def add_entity(self, entity):
        self.light_sensitive_entities.append(entity)
        if self.num_lights != 0:
            # catch the entity up with all the lights that have been added
            self.edit_shader(entity)

    def add_item(self, position, intensity):
        self.num_lights += 1
        self.lights.append(position)
        self.lights.append(intensity)
        for entity in self.light_sensitive_entities:
            # notify all the entities that a light has been added
            self.edit_shader(entity)


class ShaderManager(object):

    '''
    This class is used when you want to have an entity switch out shaders at a specific time.
    If you want to add another rule for when to switch out shaders:
    1. Add another register_XXX_shader method, make sure you call self.register() at the top
    2. In world.py have the entities that you want to follow this rule register with this shader.
    One entity can only be assigned to one shader manager at once.
    3. Create another manager class that corresponds to that rule
    4. Add a get_XXX_manager in the this class
    '''

    def __init__(self):
        self.entities = []
        self.light_manager = LightManager()

    def register(self, entity):
        if entity in self.entities:
            raise ValueError("ERROR: You've registered one entity with more that one manager, this is not allowed.")
        else:
            self.entities.append(entity)

    def unregister(self, entity):
        if entity in self.entities:
            self.entities.remove(entity)

    def register_lighting_shader(self, entity):
        self.register(entity)
        self.light_manager.add_entity(entity)

    def get_lighting_manager(self):
        return self.light_manager
