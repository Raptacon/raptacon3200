# Contributing to Raptacon3200

## Automatic Versioning

This project uses automatic versioning. You don't need to manually update version numbers - just follow the branch naming conventions and the system handles everything.

### Branch Naming

| Branch Pattern | Version Type | Example Version |
|----------------|--------------|-----------------|
| `feature/*` or `feat/*` | Alpha (testing) | `2025.2.10a1` |
| `fix/*` or `bugfix/*` | Beta (testing) | `2025.2.10b1` |
| `main` | Stable (release) | `2025.2.10` |

### Workflow

1. **Create a branch** from `main`:
   ```bash
   git checkout main
   git pull
   git checkout -b feature/my-new-feature
   ```

2. **Make your changes** and commit:
   ```bash
   git add .
   git commit -m "feat: add new feature"
   ```

3. **Push your branch**:
   ```bash
   git push -u origin feature/my-new-feature
   ```

   A test version is automatically published to PyPI (e.g., `2025.2.10a1`)

4. **Test your changes**:
   ```bash
   pip install raptacon3200==2025.2.10a1 --pre
   ```

5. **Push more commits** if needed - each push creates a new test version (`a2`, `a3`, etc.)

6. **Create a Pull Request** on GitHub and merge to `main`

   A stable version is automatically published (e.g., `2025.2.10`)

### Commit Message Format

Start your commit messages with one of these prefixes:

- `feat:` - New feature
- `fix:` - Bug fix
- `docs:` - Documentation only
- `test:` - Adding tests
- `refactor:` - Code refactoring
- `chore:` - Maintenance tasks

Examples:
```
feat: add autonomous mode selection
fix: correct motor direction for intake
docs: update README with new instructions
```

### Installing Versions

```bash
# Install latest stable release
pip install raptacon3200

# Install specific stable version
pip install raptacon3200==2025.2.10

# Install latest test/prerelease version
pip install raptacon3200 --pre

# Install specific test version
pip install raptacon3200==2025.2.10a1 --pre
```

### Quick Reference

| What you want | Command |
|---------------|---------|
| Start new feature | `git checkout -b feature/my-feature` |
| Start bug fix | `git checkout -b fix/my-bugfix` |
| Push changes | `git push` |
| Install your test version | `pip install raptacon3200==VERSION --pre` |
