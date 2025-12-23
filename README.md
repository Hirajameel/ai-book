# AI Book Documentation

Welcome to the AI Book documentation project! This repository contains comprehensive documentation about artificial intelligence concepts, techniques, and applications.

## Overview

This documentation site is built with [Docusaurus](https://docusaurus.io/), a modern static website generator designed for creating documentation sites. The site is hosted on GitHub Pages at [https://hirajameel.github.io/ai-book/](https://hirajameel.github.io/ai-book/).

## Project Structure

- `frontend-docu/` - Contains the Docusaurus documentation site source code
- `.github/workflows/` - GitHub Actions workflow for deployment
- Other files and directories as needed for the documentation

## Local Development

To run this documentation site locally:

1. Navigate to the `frontend-docu` directory:
   ```bash
   cd frontend-docu
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm run start
   ```

4. Open your browser to [http://localhost:3000](http://localhost:3000) to view the site.

## Contributing

To contribute to this documentation:

1. Fork the repository
2. Create a new branch for your changes
3. Make your changes in the `frontend-docu` directory
4. Test locally before submitting a pull request
5. Submit a pull request with your changes

## Deployment

This site is automatically deployed to GitHub Pages when changes are pushed to the `main` branch. The deployment is handled by the GitHub Actions workflow in `.github/workflows/deploy.yml`.

## Tech Stack

- [Docusaurus](https://docusaurus.io/) - Static site generator
- [React](https://reactjs.org/) - Component library
- [Node.js](https://nodejs.org/) - JavaScript runtime (requires v20+)
- [npm](https://www.npmjs.com/) - Package manager
- [GitHub Pages](https://pages.github.com/) - Hosting platform

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.