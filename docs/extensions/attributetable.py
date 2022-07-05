"""
Credit goes to discord.py and its creators for creating most of this file:
https://github.com/Rapptz/discord.py/blob/master/docs/extensions/attributetable.py
"""
import importlib
import inspect
import os
import re
from collections import OrderedDict, namedtuple
from typing import List

import sphinx.errors
from docutils import nodes
from docutils.nodes import Node
from docutils.parsers.rst.directives import unchanged  # type: ignore
from docutils.parsers.rst.directives import flag, unchanged_required
from sphinx import addnodes
from sphinx.locale import _
from sphinx.util.docutils import SphinxDirective


class attributetable(nodes.General, nodes.Element):
    pass


class attributetablecolumn(nodes.General, nodes.Element):
    pass


class attributetabletitle(nodes.TextElement):
    pass


class attributetableplaceholder(nodes.General, nodes.Element):
    pass


class cppattributetableplaceholder(nodes.General, nodes.Element):
    pass


class attributetablebadge(nodes.TextElement):
    pass


class attributetable_item(nodes.Part, nodes.Element):
    pass


def visit_attributetable_node(self, node):
    """
    Returns the starting HTML for the attribute table.
    """
    class_ = node["python-class"] if "python-class" in node else node["cpp-full-name"]
    self.body.append(f'<div class="attribute-table" data-move-to-id="{class_}">')


def visit_attributetablecolumn_node(self, node):
    """
    Returns the starting HTML for a column of the attribute table. Just a div!
    """
    self.body.append(self.starttag(node, "div", CLASS="attribute-table-column"))


def visit_attributetabletitle_node(self, node):
    """
    Returns the starting HTML for a title in the attribute table. Just a div!
    """
    self.body.append(self.starttag(node, "span"))


def visit_attributetablebadge_node(self, node):
    """
    Returns the starting HTML for a badge in the attribute table. Just a div!
    """
    attributes = {
        "class": "attribute-table-badge",
        "title": node["badge-type"],
    }
    self.body.append(self.starttag(node, "span", **attributes))


def visit_attributetable_item_node(self, node):
    """
    Returns the starting HTML for an entry in the attribute table. Just a div!
    """
    self.body.append(self.starttag(node, "li", CLASS="attribute-table-entry"))


def depart_attributetable_node(self, node):
    """
    Returns the ending HTML for the attribute table.
    """
    self.body.append("</div>")


def depart_attributetablecolumn_node(self, node):
    """
    Returns the ending HTML for a column in the attribute table.
    """
    self.body.append("</div>")


def depart_attributetabletitle_node(self, node):
    """
    Returns the ending HTML for a title in the attribute table.
    """
    self.body.append("</span>")


def depart_attributetablebadge_node(self, node):
    """
    Returns the ending HTML for a badge in the attribute table.
    """
    self.body.append("</span>")


def depart_attributetable_item_node(self, node):
    """
    Returns the ending HTML for an item in the attribute table.
    """
    self.body.append("</li>")


_name_parser_regex = re.compile(r"(?P<module>[\w.]+\.)?(?P<name>\w+)")


class PyAttributeTable(SphinxDirective):
    has_content = False
    required_arguments = 1
    optional_arguments = 0
    final_argument_whitespace = False
    option_spec = {}

    def parse_name(self, content):
        path, name = _name_parser_regex.match(content).groups()
        if path:
            modulename = path.rstrip(".")
        else:
            modulename = self.env.temp_data.get("autodoc:module")
            if not modulename:
                modulename = self.env.ref_context.get("py:module")
        if modulename is None:
            raise RuntimeError(
                f"modulename somehow None for {content} in {self.env.docname}."
            )

        return modulename, name

    def run(self):
        """If you're curious on the HTML this is meant to generate:

        <div class="attribute-table">
            <div class="attribute-table-column">
                <span>_('Attributes')</span>
                <ul>
                    <li>
                        <a href="...">
                    </li>
                </ul>
            </div>
            <div class="attribute-table-column">
                <span>_('Methods')</span>
                <ul>
                    <li>
                        <a href="..."></a>
                        <span class="py-attribute-badge" title="decorator">D</span>
                    </li>
                </ul>
            </div>
        </div>

        However, since this requires the tree to be complete
        and parsed, it'll need to be done at a different stage and then
        replaced.
        """
        content = self.arguments[0].strip()
        node = attributetableplaceholder("")
        modulename, name = self.parse_name(content)
        node["python-doc"] = self.env.docname
        node["python-module"] = modulename
        node["python-class"] = name
        node["python-full-name"] = f"{modulename}.{name}"
        return [node]


_cpp_name_parser_regex = re.compile(r"(?P<namespace>[\w.]+::)?(?P<name>\w+)")


class CppAttributeTable(SphinxDirective):
    has_content = False
    required_arguments = 1
    optional_arguments = 0
    final_argument_whitespace = False
    option_spec = {}

    def parse_name(self, content):
        path, name = _cpp_name_parser_regex.match(content).groups()
        if path:
            namespace = path.rstrip("::")
        else:
            namespace = self.env.temp_data.get("autodoc:module")
            if not namespace:
                namespace = self.env.ref_context.get("cpp:namespace")

        return namespace, name

    def run(self):
        """If you're curious on the HTML this is meant to generate:

        <div class="cpp-attribute-table">
            <div class="cpp-attribute-table-column">
                <span>_('Attributes')</span>
                <ul>
                    <li>
                        <a href="...">
                    </li>
                </ul>
            </div>
            <div class="cpp-attribute-table-column">
                <span>_('Methods')</span>
                <ul>
                    <li>
                        <a href="..."></a>
                        <span class="cpp-attribute-badge" title="decorator">D</span>
                    </li>
                </ul>
            </div>
        </div>

        However, since this requires the tree to be complete
        and parsed, it'll need to be done at a different stage and then
        replaced.
        """
        content = self.arguments[0].strip()
        node = cppattributetableplaceholder("")
        namespace, name = self.parse_name(content)
        node["cpp-doc"] = self.env.docname
        node["cpp-namespace"] = namespace
        node["cpp-class"] = name
        node["cpp-full-name"] = content
        return [node]


def build_lookup_table(env):
    # Given an environment, load up a lookup table of {full-class-name: objects}
    result = {}
    # Only for domains beginning with py
    domain = env.domains["py"]

    # Do not include domains containing these keywords - any of these domains
    # will never be in an attribute table
    ignored = {
        "data",
        "exception",
        "module",
        "class",
    }

    for (fullname, _, objtype, docname, _, _) in domain.get_objects():
        if objtype in ignored:
            continue

        # Split the full name by the last dot (ie, into the class and attribute)
        classname, _, child = fullname.rpartition(".")
        try:
            result[classname].append(child)
        except KeyError:
            result[classname] = [child]

    # Return dict of objects as described above
    return result


TableElement = namedtuple("TableElement", "fullname label badge")


def _parse_cpp_function_sig_children(children: List[Node]):
    # Remove equal sign and everything after if found
    if "=" in children:
        equal_sign = children.index("=")
        children = children[:equal_sign]

    # Remove unnecessary keywords
    to_remove = [" ", "const"]
    children[:] = [child for child in children if child not in to_remove]

    return children[-2]


def _parse_cpp_attribute_sig_children(children: List[Node]):
    # Remove equal sign and everything after if found
    if "=" in children:
        equal_sign = children.index("=")
        children = children[:equal_sign]

    # Remove array formatting
    if "]" in children:
        opening_bracket = children.index("[")
        children = children[:opening_bracket]

    children[:] = [child for child in children if child not in [" ", ""]]
    return children[-1]


def process_cppattributetable(app, doctree: Node, fromdocname):
    # Setup parser used by Doxygen to parse XML
    assert app.env is not None

    # Begin dict for class' objects
    classes = (
        {}
    )  # {'namespace::ClassName': OrderedDict((_('Attributes'), []), (_('Functions'), [])), 'ClassNameTwo': ...}
    ids = {}

    # Find all relevant C++ functions and attributes
    current_section = None

    # Throw error if user is attempting to put attribute table for enum classk
    for node in doctree.traverse(siblings=True):
        if hasattr(node, "attributes"):
            if "cpp" in node.attributes["classes"] and any(
                c in node.attributes["classes"] for c in ["class", "struct"]
            ):
                # Store current C++ struct or class section as namespace::ClassName or ClassName
                current_section = node.children[0].astext()

                # Remove class/struct prefix
                current_section = re.sub(r"^.*(class )", "", current_section)
                current_section = re.sub(r"^.*(struct )", "", current_section)

                # Remove inheritance string
                current_section = re.sub(r" : .*$", "", current_section)

                # Store goto IDs for the current section
                ids[current_section] = node.children[0].attributes["ids"][0]

                # Store basic ordered dict for attributes and functions
                classes[current_section] = OrderedDict(
                    [
                        (_("Attributes"), []),
                        (_("Functions"), []),
                    ]
                )

            elif all([c in node.attributes["classes"] for c in ["cpp", "function"]]):
                # Get the signature line of the function, where its name is stored
                try:
                    descriptions = [
                        n.astext() for n in node[0][0].children if isinstance(n, Node)
                    ]
                except IndexError:
                    continue

                # If we found a name
                if descriptions:
                    # Get the ID to link to
                    fullname = None
                    try:
                        fullname = (
                            node.children[0]
                            .children[0]
                            .children[0]
                            .attributes["ids"][0]
                        )
                    except:
                        fullname = ""

                    # Parse the function signature into just its name
                    parsed = _parse_cpp_function_sig_children(descriptions)

                    # Assign the actual table element for the func
                    badge = attributetablebadge("func", "func")
                    badge["badge-type"] = _("function")
                    if current_section:
                        classes[current_section][_("Functions")].append(
                            TableElement(fullname=fullname, label=parsed, badge=badge)
                        )

            elif all([c in node.attributes["classes"] for c in ["cpp", "var"]]):
                # Try to get signature lines for C++ variables
                try:
                    descriptions = [
                        n.astext() for n in node.children[0][0] if isinstance(n, Node)
                    ]
                except IndexError:
                    continue

                if descriptions:
                    # Parse C++ attribute signature to get only attribute name
                    parsed = _parse_cpp_attribute_sig_children(descriptions)

                    # Get the ID to link to
                    fullname = (
                        node.children[0].children[0].children[0].attributes["ids"][0]
                    )

                    # Make table element
                    if current_section:
                        classes[current_section][_("Attributes")].append(
                            TableElement(fullname=fullname, label=parsed, badge=None)
                        )

            elif isinstance(node, nodes.section):
                # Reset current section with each new section
                current_section = None

    # For each C++ attribute table requested
    for node in doctree.traverse(cppattributetableplaceholder):
        target = node["cpp-full-name"]

        # Turn the table elements in a node
        table = attributetable("")

        # Throw error if not found in list of classes
        if target not in classes:
            raise sphinx.errors.ExtensionError(
                "No C++ class or struct was found matching the "
                f"{target} target provided. Please ensure that this class is purely "
                "a C++ class or struct and not any other data structure. Additionally, "
                "ensure that proper documentation was able to be generated without "
                "the attribute table being used. If an error occurred, please open "
                "an issue on our GitHub repo."
            )

        for label, subitems in classes[target].items():
            if not subitems:
                continue
            table.append(
                class_results_to_node(label, sorted(subitems, key=lambda c: c.label))
            )

        # Turn table into node
        table["cpp-full-name"] = ids[target]

        if not table:
            node.replace_self([])
        else:
            node.replace_self([table])


def process_attributetable(app, doctree, fromdocname):
    # Build lookup table of module names and attributes
    env = app.builder.env
    lookup = build_lookup_table(env)

    # For each node in the doctree
    for node in doctree.traverse(attributetableplaceholder):
        # Info about each node
        modulename, classname, fullname = (
            node["python-module"],
            node["python-class"],
            node["python-full-name"],
        )

        # Get class results
        groups = get_class_results(lookup, modulename, classname, fullname)
        table = attributetable("")
        for label, subitems in groups.items():
            if not subitems:
                continue
            table.append(
                class_results_to_node(label, sorted(subitems, key=lambda c: c.label))
            )

        table["python-class"] = fullname

        if not table:
            node.replace_self([])
        else:
            node.replace_self([table])


def get_class_results(lookup: dict, modulename: str, name: str, fullname: str):
    # Import checked module
    module = importlib.import_module(modulename)
    cls = getattr(module, name)

    # Begin dict for class' objects
    groups = OrderedDict(
        [
            (_("Attributes"), []),
            (_("Methods"), []),
        ]
    )

    # If the class has members, let's get them! If not, it has no members, and
    # we can return nothing!
    try:
        members = lookup[fullname]
    except KeyError:
        return groups

    # For each member of the module
    for attr in members:
        attrlookup = f"{fullname}.{attr}"
        key = _("Attributes")
        badge = None
        label = attr
        value = None

        # Search up the class' inheritance tree for the implementation of the
        # desired attribute, and break when we find it!
        for base in cls.__mro__:
            value = base.__dict__.get(attr)
            if value is not None:
                break

        # If we found the value, let's document it!
        if value is not None:
            doc = value.__doc__ or ""

            # Handle async functions; give them async badge and move them to Methods
            if inspect.iscoroutinefunction(value) or doc.startswith("|coro|"):
                key = _("Methods")
                badge = attributetablebadge("async", "async")
                badge["badge-type"] = _("coroutine")

            # Handle class methods; give them the cls badge and move them to Methods
            elif isinstance(value, classmethod):
                key = _("Methods")
                label = f"{name}.{attr}"
                badge = attributetablebadge("cls", "cls")
                badge["badge-type"] = _("classmethod")

            # Handle all other methods, including decorators
            elif inspect.isfunction(value):
                if doc.startswith(("A decorator", "A shortcut decorator")):
                    # finicky but surprisingly consistent
                    badge = attributetablebadge("@", "@")
                    badge["badge-type"] = _("decorator")
                    key = _("Methods")
                else:
                    key = _("Methods")
                    badge = attributetablebadge("def", "def")
                    badge["badge-type"] = _("method")

        # Finally, with all of the compiled info, store it in the class' table
        groups[key].append(TableElement(fullname=attrlookup, label=label, badge=badge))

    # Return the class' info table
    return groups


def class_results_to_node(key, elements):
    title = attributetabletitle(key, key)
    ul = nodes.bullet_list("")
    for element in elements:
        ref = nodes.reference(
            "",
            "",
            internal=True,
            refuri=f"#{element.fullname}",
            anchorname="",
            *[nodes.Text(element.label)],
        )
        para = addnodes.compact_paragraph("", "", ref)
        if element.badge is not None:
            ul.append(attributetable_item("", element.badge, para))
        else:
            ul.append(attributetable_item("", para))

    return attributetablecolumn("", title, ul)


def setup(app):
    app.add_directive("attributetable", PyAttributeTable)
    app.add_directive("cppattributetable", CppAttributeTable)
    app.add_node(
        attributetable, html=(visit_attributetable_node, depart_attributetable_node)
    )
    app.add_node(
        attributetablecolumn,
        html=(visit_attributetablecolumn_node, depart_attributetablecolumn_node),
    )
    app.add_node(
        attributetabletitle,
        html=(visit_attributetabletitle_node, depart_attributetabletitle_node),
    )
    app.add_node(
        attributetablebadge,
        html=(visit_attributetablebadge_node, depart_attributetablebadge_node),
    )
    app.add_node(
        attributetable_item,
        html=(visit_attributetable_item_node, depart_attributetable_item_node),
    )
    app.add_node(attributetableplaceholder)
    app.add_node(cppattributetableplaceholder)
    app.connect("doctree-resolved", process_attributetable)
    app.connect("doctree-resolved", process_cppattributetable)
