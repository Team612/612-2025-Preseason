# 612-Template
Captains: use this to start new projects and update to your necessities.

# Branches
- main branch along with its feature branches are protected. **Note:** must make branch protection rules for each feature branch, meaning experimental branches get made and protected before actual code gets pushed

# Settings
- Options - Under the Merge button heading, unchecked "Allow rebase merging", checked "Allow auto-merge" (member can select "Enable auto-merge" after creating a pr/when waiting for approval), checked "Automatically delete head branches" (can restore if necessary for up to 30-Days)
- Branches - Created a branch protection rule with main branch as Branch name pattern with these checked: "Require a pull request before merging", "Require approvals", "Require review from Code Owners", "Require status checks to pass before merging", "Require branches to be up to date before merging", "Require linear history", and "Include administrators"

# Github Actions
- Created a new "Java CI with Gradle" action file. Under main branch, the ```branches:``` option reads ```[main]``` only but under the feature-template branch, the ```branches:``` option reads ```[main, feature-template].``` **Note:** change ```feature template``` name to whatever the branch becomes renamed as.
- Created project-setup.yml file for project automation. **Note:** authors must replace ```project``` url with new repository's project's url.
- **Test Cases:** Example test case created in src/test/java/TestTemplate.java. Future test cases that check actual robot code logic should be placed in ```java``` folder here.

# Project Template
- Created a new automated kanban project.
- To-do setup: Move issues here when "Newly added"
- In progress setup: Move issues here when "Reopened", Move pull requests here when "Newly opened", "Reopened" and "Pending approval by reviewer"
- Done setup: Move issues here when "Closed", Move pull requests here when "Merged"

# Webhooks
- Discord: under server settings, navigated to the "Integrations" tab. Created a new webhook and copied the URL. Under repository settings, navigated to "Webhooks" tab, created a new webhook. Copied webhook URL to "payload URL" and appended "/github" to the end of the URL. Under "Content Type" dropdown, switch to "application/json". Requested that the repository sends all events to webhook. Created GitHub webhook. 
