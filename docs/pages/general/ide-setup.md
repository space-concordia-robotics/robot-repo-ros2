# IDE/Editor Setup

VSCode is the recommended editor, however if you like CLion, you also have the option to use that.

## VSCode

### Extensions

The following extensions are required:

- [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python) ([Open VSX](https://open-vsx.org/extension/ms-python/python))
- [Python Debugger](https://marketplace.visualstudio.com/items?itemName=ms-python.debugpy) ([Open VSX](https://open-vsx.org/extension/ms-python/debugpy))
- [Pylance](https://marketplace.visualstudio.com/items?itemName=ms-python.vscode-pylance) (Not on Open VSX[^not-on-open-vsx])
- [C/C++ Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack)
  includes the following necessary extensions:
    - [CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools) (Not on Open VSX[^not-on-open-vsx])
    - [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) (Not on Open VSX[^not-on-open-vsx])
- [clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd) ([Open VSX](https://open-vsx.org/extension/llvm-vs-code-extensions/vscode-clangd))
- [Ruff](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff) ([Open VSX](https://open-vsx.org/extension/charliermarsh/ruff))
- [ty](https://marketplace.visualstudio.com/items?itemName=astral-sh.ty) ([Open VSX](https://open-vsx.org/extension/astral-sh/ty))
- [EditorConfig](https://marketplace.visualstudio.com/items?itemName=EditorConfig.EditorConfig) ([Open VSX](https://open-vsx.org/extension/EditorConfig/EditorConfig))

[^not-on-open-vsx]: Must be downloaded & installed manually, see: https://stackoverflow.com/a/79565372

### Configuration

- `editor.formatOnSave`: set to true
- `editor.formatOnSaveMode`: set to "modified"
- `C_Cpp.doxygen.generatedStyle`: set to `#!cpp /**`
- `C_Cpp.codeAnalysis.clangTidy.enabled`: set to true

## CLion

!!! note inline end

    Note that clion must always be launched from the command line to ensure that it is able to properly locate all the ROS headers,
    see: https://www.jetbrains.com/help/clion/ros2-tutorial.html#launch-clion

CLion is particularly annoying to configure properly for a ROS project.

Read the official [ROS2 Setup Tutorial](https://www.jetbrains.com/help/clion/ros2-tutorial.html). Ignore the entire "How to build a package" section.

Once you have imported the compilation database, the `.idea` directory is currently located under the `build` directory. In order to fix that, you must do the
following:

1. Close CLion
2. Move `build/.idea` to `.idea`, e.g. in the root directory run `#!bash mv build/.idea/ .idea/`.
3. Modify the `.idea/misc.xml` file in a text editor (e.g. `nano`) with the following changes:
    - Modify the `CompDBSettings` component, and replace all instances of `$PROJECT_DIR$` with `$PROJECT_DIR$/build`.\
      After modification, it should look like this:
      ```xml title=".idea/misc.xml"
      <component name="CompDBSettings">
        <option name="linkedExternalProjectsSettings">
          <CompDBProjectSettings>
            <option name="externalProjectPath" value="$PROJECT_DIR$/build" />
            <option name="modules">
              <set>
                <option value="$PROJECT_DIR$/build" />
              </set>
            </option>
          </CompDBProjectSettings>
        </option>
      </component>
      ```
    - Modify the `CompDBWorkspace` component, and remove the `contentRoot` tag.\
      After modification, it should look like this:
      ```xml title=".idea/misc.xml"
      <component name="CompDBWorkspace" PROJECT_DIR="$PROJECT_DIR$" />
      ```
4. Modify the `.idea/workspace.xml` file in a text editor with the following changes:
    - Modify the `CompDBLocalSettings` component, and replace all instances of `$PROJECT_DIR$` with `$PROJECT_DIR$/build`
      (also duplicate the `projectSyncType` entry and only update one of them), as well as all instances of `build` with the directory you cloned to (e.g.
      `build` -> `robot-repo-ros2`).\
      After modification, it should look like this:
      ```xml { title=".idea/workspace.xml" .annotate }
      <component name="CompDBLocalSettings">
        <option name="availableProjects">
          <map>
            <entry>
              <key>
                <ExternalProjectPojo>
                  <option name="name" value="robot-repo-ros2" /><!--(1)!-->
                  <option name="path" value="$PROJECT_DIR$/build" />
                </ExternalProjectPojo>
              </key>
              <value>
                <list>
                  <ExternalProjectPojo>
                    <option name="name" value="robot-repo-ros2" /><!--(2)!-->
                    <option name="path" value="$PROJECT_DIR$/build" />
                  </ExternalProjectPojo>
                </list>
              </value>
            </entry>
          </map>
        </option>
        <option name="projectSyncType">
          <map>
            <entry key="$PROJECT_DIR$" value="RE_IMPORT" /><!--(3)!-->
            <entry key="$PROJECT_DIR$/build" value="RE_IMPORT" /><!--(4)!-->
          </map>
        </option>
      </component>
      ```
        1. Updated from `build` to `robot-repo-ros2`
        2. Updated from `build` to `robot-repo-ros2`
        3. Remember to leave this one as `$PROJECT_DIR$`.
        4. Actually, I don't think this one is fully necessary. I *think* it might generate it for you, but I'm not 100% sure.
    - Modify the `ExternalProjectsData` component, and duplicate the `projectState`, then update one of them and replace `$PROJECT_DIR$` with
      `$PROJECT_DIR$/build`\
      After modification, it should look like this:
      ```xml { title=".idea/workspace.xml" .annotate }
      <component name="ExternalProjectsData">
        <projectState path="$PROJECT_DIR$"><!--(1)!-->
          <ProjectState />
        </projectState>
        <projectState path="$PROJECT_DIR$/build"><!--(2)!-->
          <ProjectState />
        </projectState>
      </component>
      ```
        1. Remember to leave this one as `$PROJECT_DIR$`.
        2. Actually, I don't think this one is fully necessary. I *think* it might generate it for you, but I'm not 100% sure.
5. Open CLion in the project directory (*not* in the build directory).

### Plugins

The following plugins are required:

- [Python LSP](https://plugins.jetbrains.com/plugin/29422-python-lsp)
- [EditorConfig](https://plugins.jetbrains.com/plugin/7294-editorconfig)

### Configuration

Once all the required plugins are installed, you change the following configuration options:

- **Languages & Frameworks** > C/C++ > Clangd > Show errors and warnings from clangd: enable
- **Languages & Frameworks** > C/C++ > Clangd > Use clang-tidy via clangd: enable
- **Tools** > Ruff > Enable: enable
- **Tools** > Ruff > Features > Inspections: enable
- **Tools** > Ruff > Features > Import optimizer: enable
- **Tools** > Ruff > Execution mode: set to "Path"
- **Tools** > ty > Enable: enable
- **Tools** > ty > Features > Inspections: enable
- **Tools** > ty > Features > Completions: enable
- **Tools** > ty > Features > Inlay Hints: enable
- **Tools** > ty > Features > Documentation: enable
- **Tools** > ty > Execution mode: set to "Path"

