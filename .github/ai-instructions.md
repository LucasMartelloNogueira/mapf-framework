- any time that you need to create a plan.md file for a new feature, follow the instructions below:
  
  1. Analise the existing codebase
    - Review the current code to understand the architecture, development patterns, and
      libraries used.
    - Identify reusable components, hooks, and structures that can be leveraged.
  
  2. Search for libs and documentations
    - Explore the libraries already in use within the project to ensure you are utilizing
      their functionalities in the best possible way.
    - Research new libraries that could add value to the development, always considering
      compatibility with the existing ecosystem.
  
  3. Implementation plan elaboration
    - Create a detailed document containing:
      - List of documents and files that need to be changed or created.
      - Libraries to be used and justifications for their choice.
      - Code snippets exemplifying how to implement the main functionalities.
  
  4. Plan storage
    - Save the document in the folder `specs/{id}-{title}/plan.md` for easy access and review
      by other team members.
    - save the plan in english, even if the prompt is in another language

  5. Save prompt
    - store the user prompt in a section called "prompt". 
    - the prompt saved must be the entire prompt given by the user and without any modifications
  
  6. Save data for audit
    - the first thing saved in each plan is a datetime of when if was created, the author (the user loged in vscode / copilot) and the time it was last updated

  7. Save alterations to the plan
    - after the user makes adjustments to the plan, save the adjustments in a section called "adjusments"
    - The "adjustments" must contain subsections named "adjustment 1" for the first correction, "adjustment 2" for the second correction and so on
    - Each subsection "ajustment x" must contain the datetime of the correction, the whole prompt given by the user, without any modifications, and a list with what changed with before and after values
    - Modify the last updated field in the plan after each adjustment

  8. Get context
    - before reading the user prompt, read the files `mapf.md` to get context of the problem


- Any time that you need to create a spec.md file for a new feature, follow the instructions below:
  
  1. Requirement gathering
    - Use the plan in the specs/{id}-{title}/plan.md as a basis to gather and define the
      requirements for the feature.
    - the user should specify which plan file should be used to create the spec.
    - Define the objective, scope, and step-by-step instructions to implement the feature,
      ensuring clarity and completeness.
  
  2. Spec storage
    - Save the document in the folder `specs/{id}-{title}/spec.md` for easy access and reference
      during development.
    - save the spec file in english, even if the prompt is in another language

- Any time that you need to implement a new feature, follow the instructions below:
  
  1. Review the spec
    - Thoroughly read the spec located in `specs/{id}-{title}/spec.md` to understand the
      requirements and expectations for the feature.
    - the user should specify which spec file should be used for the implementation.
  
  2. Implementation
    - Follow the step-by-step instructions provided in the spec to implement the feature.
    - Ensure that all code adheres to best practices, including SOLID principles, modularity,
      and reusability.
    - Include error handling, input validation, and unit tests to verify functionality.
  
  3. Testing and validation
    - After implementation, run all tests to ensure that the new feature works as expected
      and does not introduce any regressions.
    - Validate that the feature meets all requirements outlined in the spec.
  
  4. Documentation
    - Document any new components, hooks, or significant changes made during the implementation
      for future reference and maintenance.
    - Use a directory called `docs/` at the root of the project to store documentation files.
  
  5. Save data for audit
    - the first thing saved in each plan is a datetime of when if was created, the author (the user loged in vscode / copilot), the time it was last updated and the AI model used

  6. Save alterations to the plan
    - after the user makes adjustments to the plan, save the adjustments in a section called "adjusments"
    - The "adjustments" must contain subsections named "adjustment 1" for the first correction, "adjustment 2" for the second correction and so on
    - Each subsection "ajustment x" must contain the datetime of the correction, the whole prompt given by the user, without any modifications, and a list with what changed with before and after values
    - Modify the last updated field in the plan after each adjustment

- Any time that you need to create some feature, use the generate plans and spec files previously generated as reference !!!.

- If the user doesn't provide all the necessary information to create the plan, spec, or implement the feature, ask for the missing details before proceeding.

- If the user requests changes to an existing plan, spec, or feature implementation, follow the same steps outlined above to ensure consistency and quality.

- If at any point you identify potential improvements or optimizations in the existing codebase while creating plans, specs, or implementing features, document these suggestions using "TODO: <comment>" for future consideration by the development team.

- when the user request for feature without spec or plan, follow the steps bellow:

  1. create a plan.md for this feature, and save the path for it in ./tmp.md
  2. clean up you context window
  3. create a spec.md follow the plan.md which the path is in ./tmp.md file, save the path for in ./tmp.md
  4. clean up you context window
  5. implement the spec.md in ./tmp.md, and remove the tmp.md file