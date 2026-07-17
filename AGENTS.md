# Project Agent Workflow

After completing every requested change in this repository, finish the full
publication workflow unless the user explicitly asks not to publish:

1. Run checks appropriate to the change and confirm they pass.
2. Review the working tree and stage only files that belong to the request.
   Never include unrelated user changes.
3. Commit the requested change with a concise, descriptive commit message.
4. Push the current branch to `origin` and set its upstream when needed.
5. Deploy Firebase Hosting with:

   ```sh
   firebase deploy --only hosting --project grapplemap-solver-vr
   ```

6. Verify the push and deployment succeeded, then report the commit, branch,
   checks, deployment status, and live Hosting URL to the user.

If authentication, tests, push, or deployment fails, investigate and retry safe
recoverable failures. Do not claim completion until the entire workflow succeeds;
report any remaining blocker precisely.
