import os
import collections


class ShaderReader(object):
    @classmethod
    def read_shader(self, shader_text):
        '''Given an iterable containing the text of a shader file,
        parse it to produce a dictionary of usable subshaders
        '''
        shader_dict = {}
        current_shader_hierarchy = None

        current_shader_text = ''
        for line in shader_text:
            if line.startswith('>'):
                if current_shader_hierarchy is not None:
                    self.update(
                        shader_dict,
                        self.recursive_dictionary(
                            current_shader_hierarchy,
                            current_shader_text
                        )
                    )
                    current_shader_text = ''

                current_shader_hierarchy = line[1:].strip().split(':')

            # Don't parse any lines if the current shader has not been specified
            elif current_shader_hierarchy is None:
                continue

            # Start assembling text
            else:
                current_shader_text += line

        # Catch anything we missed
        if current_shader_hierarchy is not None:
            self.update(
                shader_dict,
                self.recursive_dictionary(
                    current_shader_hierarchy,
                    current_shader_text
                )
            )

        return shader_dict

    @classmethod
    def update(self, _dict, _update):
        for k, v in _update.iteritems():
            if isinstance(v, collections.Mapping):
                r = self.update(_dict.get(k, {}), v)
                _dict[k] = r
            else:
                _dict[k] = _update[k]
        return _dict

    @classmethod
    def recursive_dictionary(self, path, text):
        dct = {}
        if len(path) == 0:
            return text
        # Recurse until we've made a complete entity
        dct[path[0]] = self.recursive_dictionary(path[1:], text)
        return dct


class Shaders(object):
    '''A Shaders class, which automatically contains all of the shaders
        in the shaders folder
    WTF?
        shaders.Shaders.passthrough is a dictionary containing the point:vertex and point:fragment shaders

    But there's no code in this class...
        Python: No parents, no rules.
        The files are parsed, and the attributes programmatically added to this class in the coming lines
         that is what 'setattr' does.
    '''


filepath = os.path.dirname(os.path.realpath(__file__))
for _file in os.listdir(filepath):
    file_name = os.path.split(_file)[-1]
    name, ext = os.path.splitext(file_name)
    if ext == '.glsl':
        path_to_file = os.path.join(filepath, _file)
        setattr(Shaders, name, ShaderReader.read_shader(file(path_to_file)))


if __name__ == '__main__':
    print Shaders.passthrough['debug']['vertex']
    print '---------'
    print Shaders.passthrough['debug']['fragment']
    print '========='
    print Shaders.passthrough['mesh']['vertex']
    print '---------'
    print Shaders.passthrough['mesh']['fragment']
