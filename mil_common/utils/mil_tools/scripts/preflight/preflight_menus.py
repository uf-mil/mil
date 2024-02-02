from PyInquirer import prompt
from rich.console import Console


def display_start_menu():
    console = Console()

    # Title
    console.print(
        "[bold green]Preflight Program - Autonomous Robot Verification[/bold green]\n",
    )

    # Description
    console.print(
        "Welcome to the Preflight Program, a tool inspired by the preflight checklists used by pilots before "
        "flying a plane. This program is designed to verify the functionality of all software and hardware "
        "systems on your autonomous robot. It ensures that everything is in working order, allowing you to "
        "safely deploy your robot with confidence.\n",
    )

    # Authors section
    console.print("\n[italic]Authors:[/italic]")
    console.print("Keith Khadar")
    console.print("Anthony Liao")
    console.print("Joshua Thomas\n")

    # Menu options
    start_menu = [
        {
            "type": "list",
            "name": "mode selection",
            "message": "Menu",
            "choices": [
                "Run Preflight Full Test",
                "View Report",
                "Run Specific Test",
                "View Documentation",
            ],
        },
    ]
    option = prompt(start_menu)
    return next(iter(option.values()))
