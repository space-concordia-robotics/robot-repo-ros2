# Git Etiquette

These are some best-practices to follow when using git

## Commits

- Commit frequently.
- Keep commits relatively small. Try to keep commits to "one thing", split multiple changes into separate commits.

    <!--- @formatter:off -->
    !!! tip inline end "Force with lease"

        If you need to do a force push, you can use `--force-with-lease` to make sure you don't accidentally overwrite someone else's changes.
    <!-- @formatter:on -->

- Commits should be 'complete'. That means, a commit should be able to be compiled and should not have any warnings/errors. (1)\
  A commit doesn't need to have a *full* implementation of a feature, you can do that across multiple commits, but at the very least it should not cause any
  build errors.
  { .annotate }

    1. This is very useful if you ever encounter an issue that wasn't there before and need to go back and bisect the git history.

- Don't commit large binary files. If you do accidentally commit a binary file, then you can do a git rebase and then force push.

## Commit Messages

- The first line of the commit message is the subject, and any lines after that are the body of the commit message.
  There should always be one line between the subject and the body.
- Explain the reasoning behind your changes in your commit message\
  if it's not super obvious why something was changed, then write an explanation in the commit message.
  People can come back to it later if they're wondering why a certain thing was done.\
  It's not always necessary to explain everything in the commit message, but sometimes it can be helpful.
- Be succinct. It is good to add detail, but try not to be *too* chatty. Not everything needs to be explained.
- Keep the subject to 50 characters and the body lines to 72 characters.
    - If the subject exceeds 50 lines, then you might want to split up the commit.
    - If a body line exceeds 72 characters, then wrap it.
- Only use ASCII characters in your commit message

??? success "Good Examples"

    Here are some examples of good commit messages:

    ```
    Replace ament_python with ament_cmake & ament_cmake_python

    ament_python is incredibly slow for some reason, ament_cmake is quite a
    bit faster.
    In personal testing, build times decreased from 3.0 seconds to 1.5
    seconds if nothing needed to be rebuilt.
    ```

    ```
    Add --ros-encoding and type the ROS additions in main.py

    - --ros-encoding bgr8|rgb8|mono8 (mono8 is a third of the size)
    - ArUco output and forwarding moved into small typed helpers, so
      build_pipeline() and the session loop gain no complexity
    - Only dai.ImgFrame messages are forwarded to the ROS sink
    - ruff and ty report nothing new compared with main
    ```

    ```
    Minify docs when built by CI
    ```

    ```
    Add README to docs folder
    ```

    ```
    Add can-utils dependency and update telemetry handling

    - Added can-utils to CMakeLists.txt dependencies.
    - Updated battery_board.hpp to remove unnecessary include.
    - Corrected frame ID handling in battery_board.cpp for rail telemetry.
    - Fixed variable names and types for consistency in battery and rail telemetry.
    - Modified parser.hpp to include vector instead of array.
    - Added TCU_STATUS to prefixes.hpp.
    ```

??? danger "Bad Examples"

    Here are some examples of bad commit messages:

    ```
    rollback
    ```
    Too value. Why was it rolled back?

    ---

    ```
    test
    ```
    Bad commit message.

    ---

    ```
    Trained Weights
    ```
    What are these weights from? how were they trained? etc.

    ---

    ```
    Some additional work on nav2
    ```
    What additional work?

    ---

    ```
    Add autonomy stuff

    I really need to commit more frequently
    ```
    A sign I really should have committed sooner.

    ---

    ```
    Remove wheels controller code
    ```
    Why was it removed? What was removed?

    ---

    ```
    Changed the arm_test.py launch file
    ```
    Bad subject. What was changed?

    ---

    ```
    Update .gitignore to include WSL metadata

    Add WSL metadata to .gitignore
    ```
    Body just repeats the subject (no body is necessary here)

    ---

    ```
    feat: add scripts
    ```
    What scripts? What do they do?

## Branches

Branches should only really do 'one thing'.
Try and avoid making 'super' branches; that is, branches which change a lot of things in a lot of different areas.
The larger a branch becomes, the harder it is to merge.
It takes more time to review it, requesting changes in one area delays getting changes in another area merged, etc.
