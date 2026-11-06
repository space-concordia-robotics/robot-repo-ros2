# Contributing

We are always open to new people joining the team and helping out.

## Contribution Process

???+ tip inline end "Branch Naming"

    When naming the branch, please use only lowercase letters and dashes.
    The name of the branch should begin with the type of change, followed by the name.
    The name should make it relatively obvious what the branch is for.
    For example, `feature/add-cameras`, `fix/broken-build`, `refactor/ik-cleanup`, etc.
    Sometimes it might also be beneficial to add a category: `feature/autonomy/improve-pathing`.

If you would like to get started with contributing, here is the general process:

1. Request a task from your lead or be assigned one
2. Fork the github repository
3. Create a new branch
4. Complete your task
5. Create a pull request for your branch (1)
   { .annotate }

    1. If your task is a relatively small task, then it should be fine to wait until you're done to create the pull request.\
       However, if it's a larger task, then it is recommended that you create a draft pull request so that you can get feedback sooner.\
       When you're done, you can convert your pull request from a draft to a normal pull request.

6. Wait for reviews
7. If there are any issues, then solve them (1)
   { .annotate }

    1. Depending on the scope of your changes, sometimes this can be a long process.

8. Once all the issues are resolved, your code will be merged into main

## Expectations

When contributing, we expect that you will do the following:

- Follow our [code standards][code standards].\
  Before opening a PR (and ideally before you commit), make sure to use a linter and a formatter for your code.
- Follow general [git etiquette][git etiquette].
- Understand your changes.\
  You should be able to explain why your made certain changes and be able to answer questions about them.\
  If you need help with certain things, then feel free to ask.\
  Please do not just use an LLM to generate code and submit it without looking at it.
- Test your code before opening a pull request.
- Respond to and integrate any feedback. We will work with you to get your changes ready to be merged.
- Have fun!

[code standards]: ../code-standards.md
[git etiquette]: git-etiquette.md
