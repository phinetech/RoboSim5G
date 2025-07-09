This project develops **gNB and UE plugins for Gazebo SIM 6 (Ignition Fortress)** using **C++ and ROS 2**. Please follow these conventions to keep the codebase clean, consistent, and collaborative.

### 📁 General Structure

- Place plugin code under `phine-gz-ros/src/phine_plugins/src` and `phine-gz-ros/src/phine_plugins/include/` folders .
    
- ROS 2 nodes and interfaces follow standard package layout (`launch/`, `msg/`, `srv/`, etc.).
    
#### 🔢 Indentation

- Use **Tabs** for indentation (not spaces).
    
- Configure your editor to insert a real **Tab character (`\t`)**, not spaces.
    
- Recommended: Set Tab width to **2 or 4 spaces** visually — but still use actual Tab characters.
### 📐 Style & Formatting

- **Language**: C++17 (or newer if agreed upon)
    
- **Style Guide**: Follow ROS 2 C++ style (Google C++ style is a good fallback)
    
- **Brace Style**: Attach to function/loop signature
    
- Use `clang-format 14` (`.clang-format` file provided), configured to use tabs, to auto-format code:
    
```bash    
    clang-format -i your_file.cpp
```

### 🔤 Naming Conventions

|Element|Convention|Example|
|---|---|---|
|Variables|`snake_case`|`ue_status`|
|Functions|`camelCase()`|`startPlugin()`|
|Classes|`CamelCase`|`UeControllerPlugin`|
|Constants|`ALL_CAPS`|`DEFAULT_TIMEOUT`|
|ROS Topics|`snake_case`|`/ue/status`|

### 📄 Documentation

- Add JSDoc-like Style comments for classes, functions, and complex logic.
    
- Public headers should have a short comment explaining their purpose.

### Branching

- `main`: Stable branch, protected
    
- `dev`: Development branch, protected
    
- Feature branches: `feature/<short-description>`
    
- Bugfix branches: `bugfix/<short-description>`
    

---

### Merge Requests (MRs)

- Target `dev` branch
    
- Write a clear description (what & why)
    
- Link issues (e.g., `Closes #23`)
    
- Assign a reviewer/maintainer
    

---

### Commit Messages

Use this format:


```cpp
<type>: <short summary>  

[optional body: what changed and why]

Closes #issue
```
Types: `feat`, `fix`, `docs`, `refactor`, `test`, `chore`

Example:

```vbnet
feat: add UE signal control service

Implements start/stop toggle for UE using ROS 2 service.

Closes #12`
```
---

## ✅ Tools

- **Formatter**: `clang-format` (configured for Tabs)
    
- **Build**: `colcon build`
    
- **Testing**: Run with Gazebo + ROS 2
    
- **CI**: GitLab CI (formatting, build test)