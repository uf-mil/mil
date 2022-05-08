'use-strict';

let lightDarkControl;

class LightDarkMode {
  constructor() {
    this.value = null;
    this.getAutomaticValue();
  }

  getAutomaticValue() {
    this.value = localStorage.getItem('mil-docs-light-dark');
    if (this.value == null || this.value == 'automatic') {
      this.changeAuto();
    } else if (this.value == 'dark') {
      this.changeDark();
    } else if (this.value == 'light') {
      this.changeLight();
    }
  }

  changeDark() {
    this.value = 'dark';
    document.documentElement.setAttribute('data-theme', this.value);
    localStorage.setItem('mil-docs-light-dark', this.value);
    document.getElementById('mil-light-dark-icon').getElementsByTagName('span')[0].innerHTML = 'nights_stay';
    document.getElementById('mil-light-dark-icon').getElementsByTagName('span')[0].title = "The site is displayed using a dark mode theme.";
  }

  changeLight() {
    this.value = 'light';
    document.documentElement.setAttribute('data-theme', this.value);
    localStorage.setItem('mil-docs-light-dark', this.value);
    document.getElementById('mil-light-dark-icon').getElementsByTagName('span')[0].innerHTML = 'light_mode';
    document.getElementById('mil-light-dark-icon').getElementsByTagName('span')[0].title = "The site is displayed using a light mode theme.";

    // No need to listen to when user changes their light/dark preference
    window.matchMedia('(prefers-color-scheme: dark)').removeEventListener('change', this.changeAuto);
  }

  changeAuto() {
    this.value = 'automatic';
    if (window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches) {
      document.documentElement.setAttribute('data-theme', 'dark');
    } else if (window.matchMedia && window.matchMedia('(prefers-color-scheme: light)').matches) {
      document.documentElement.setAttribute('data-theme', 'light');
    }
    localStorage.setItem('mil-docs-light-dark', this.value);
    document.getElementById('mil-light-dark-icon').getElementsByTagName('span')[0].innerHTML = 'autorenew';
    document.getElementById('mil-light-dark-icon').getElementsByTagName('span')[0].title = "Dark mode/light mode is triggered based on your browser's preference.";

    // Respond to user changing their preference
    window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', this.changeAuto);
  }

  rotate() {
    if (this.value == 'automatic') {
      this.changeLight();
    } else if (this.value == 'light') {
      this.changeDark();
    } else if (this.value == 'dark') {
      this.changeAuto();
    }
  }
}

// Can be called globally to change between light/dark
function rotateLightDark() {
  lightDarkControl.rotate();
}

document.addEventListener('DOMContentLoaded', () => {
  lightDarkControl = new LightDarkMode();
});
