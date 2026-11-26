<!--
  Please make the title of your PR informative.
  It should summarize the PR without being too long.
-->

## Overview

### Summary

<!--
  Briefly describe the changes made.
  Should be around 1-4 sentences.

  This section is mandatory and must be filled out.
-->

### References

<!--
  Reference any relevant issues/PRs here with closing keywords: https://docs.github.com/en/issues/tracking-your-work-with-issues/using-issues/linking-a-pull-request-to-an-issue
  e.g. "Closes #123"
  For smaller changes, an issue is not strictly necessary.
  However for larger features or changes, please create an issue first so that the proposed changes can be discussed.

  This section is not mandatory.
  If this section is not applicable, then please remove it.
-->

### Type of Change

<!--
  What type of change is this? (you can select multiple)
  You can mark off items by adding an x inside the brackets, e.g.
  - [x] Feature

  This section is mandatory and must be filled out.
-->

- [ ] Feature
- [ ] Bug fix
- [ ] Refactoring (i.e. no functional changes)
- [ ] Documentation update
- [ ] Performance improvement
- [ ] Build system
- [ ] Configuration/parameters update
- [ ] Other

## Implementation

### Description

<!--
  What does this PR do, and why?
  What problem does it solve?
  The length of the description should be roughly proportional to the changes made.

  If there were any important design decisions made, then please explaine the rationale here.
  If it was relatively straightforward and there was really only one way to do it,
  then you do not need to explain that.

  This section is mandatory and must be filled out.
-->

### LLM Disclosure

<!--
  You are allowed to used LLM-based tools to help with writing code,
  however it is important that you understand any changes you have made.

  If you used any LLM-based tools to help write or understand the code, you should disclose that here.
  Please say what tools (and what specific models) were used and what they were used for.

  You may not use LLM-based tools to write documentation, documentation must be written by a human.

  This section is mandatory and must be filled out if you used any LLM-based tools.
  If you did not use any LLM-based tools, then you may remove this section.
-->

### Changes

<!--
  A list of things that you have changed or are going to change.
  If you are opening a draft PR, then you can check these off as you add them.
  If this PR is not a draft PR, then EVERYTHING should be checked off before opening the PR.

  If your changes are not yet ready to merge, then please open a draft PR.

  This section is mandatory and must be filled out.
-->

- [ ] <!-- First thing -->
  - [ ] <!-- Sub-item 1 of the first thing -->
  - [ ] <!-- Sub-item 2 of the first thing -->
- [ ] <!-- Second thing -->
- [ ] <!-- Third thing -->

### Additional Notes

<!--
  Any additional information or context relevant to this PR.

  Here are some examples of things that could be included:
  - new dependencies
  - configuration changes
  - known limitations
  - any needed follow-up work
  - screenshots

  This section is not mandatory.
  If this section is not applicable, then please remove it.
-->

## Testing

<!--
  What can reviewers do to test your changes?
  What tests have you performed before submitting this PR?

  Have you tested on the rover, or only in simulation?
  Have you written any unit tests?

  If there is something you couldn't test or did not test, please mention it and say why.

  This section is not mandatory.
  If this section is not applicable (e.g. documentation update), then please remove it.
-->

## Checklist

<!--
  Complete this checklist before submitting your pull request.
  You can mark off items by adding an x inside the brackets, e.g.
  - [x] ...

  If you are making a draft PR, then the tests item may be unchecked,
  but you should check it before converting to a normal PR.

  This section is mandatory and must be filled out.
-->

- [ ] I am the original author of the code, I have the necessary rights and permissions to contribute it, or I have verified its use and distribution are permitted under the applicable license.
- [ ] I have made sure to follow the [repository code standards](https://space-concordia-robotics.github.io/general/code-standards/)
  - [ ] I have formatted my code according to the project's style guidelines (`clang-format` for C++, `ruff` or your IDE's formatter for python)
  - [ ] I have used `clang-tidy` and `ruff`/`ty` to check my code and there are no new warnings
- [ ] My PR is focused on one thing and does not change many disconnected things.
  <!--
    If you have a lot of changes to make to many different areas, then it is okay to open several PRs.
    For example, if you find a bug in something else while working on one thing, then open a separate PR for that bug fix.
  -->
- [ ] I fully understand the code in this PR and am able to explain it in my own words.
- [ ] I have reviewed my code before submitting
- [ ] I have updated any relevant documentation for my code
  <!--
    The documentation for most things can be found in the `docs/` folder or the package's `README.md`.
    Not every change requires updating the documentation.
  -->
- [ ] I have updated any relevant tests and added tests as necessary
  <!--
    Due to the nature of what we're doing, not everything can be easily tested.
    However, please try to write tests wherever possible.
  -->
- [ ] I did not use any LLM-based tools to generate the description of this PR
  <!--
    This means, the *description* of the PR was written by a human.
    You should not use any LLM-based tools for writing the description of the PR,
    as you are expected to be able to understand the changes you made.
  -->
- [ ] I have disclosed any use of LLM-based tools for writing code.
