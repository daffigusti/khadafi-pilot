#!/bin/bash

# Sunnypilot Branch Update Script
# Fetches dev-c3-new from sunnypilot remote, creates local branch, and cherry-picks changes from dev-e5n

set -e  # Exit on any error

# Configuration
REMOTE_NAME="sunnypilot"
SOURCE_BRANCH="dev-c3-new"
CURRENT_BRANCH="dev-e5n"
TARGET_BRANCH="dev-c3-new"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if we're in a git repository
check_git_repo() {
    if ! git rev-parse --git-dir > /dev/null 2>&1; then
        print_error "Not in a git repository!"
        exit 1
    fi
}

# Function to check if working directory is clean
check_clean_working_dir() {
    if ! git diff-index --quiet HEAD --; then
        print_error "Working directory is not clean. Please commit or stash your changes."
        git status --short
        exit 1
    fi
}

# Function to check if remote exists
check_remote() {
    if ! git remote get-url "$REMOTE_NAME" > /dev/null 2>&1; then
        print_error "Remote '$REMOTE_NAME' not found!"
        print_status "Available remotes:"
        git remote -v
        exit 1
    fi
}

# Function to fetch latest updates
fetch_updates() {
    print_status "Fetching $SOURCE_BRANCH from $REMOTE_NAME..."
    if git fetch "$REMOTE_NAME" "$SOURCE_BRANCH"; then
        print_success "Successfully fetched $SOURCE_BRANCH from $REMOTE_NAME"
    else
        print_error "Failed to fetch $SOURCE_BRANCH from $REMOTE_NAME"
        exit 1
    fi
}

# Function to backup existing branch if it exists
backup_existing_branch() {
    if git show-ref --verify --quiet "refs/heads/$TARGET_BRANCH"; then
        local backup_name="${TARGET_BRANCH}-backup-$(date +%Y%m%d_%H%M%S)"
        print_warning "Local branch '$TARGET_BRANCH' already exists"
        print_status "Creating backup as '$backup_name'..."
        
        if git branch "$backup_name" "$TARGET_BRANCH"; then
            print_success "Backup created: $backup_name"
            
            # Delete the old branch
            git branch -D "$TARGET_BRANCH"
            print_status "Deleted old '$TARGET_BRANCH' branch"
        else
            print_error "Failed to create backup branch"
            exit 1
        fi
    fi
}

# Function to create new branch from remote
create_new_branch() {
    print_status "Creating new branch '$TARGET_BRANCH' from '$REMOTE_NAME/$SOURCE_BRANCH'..."
    
    if git checkout -b "$TARGET_BRANCH" "$REMOTE_NAME/$SOURCE_BRANCH"; then
        print_success "Successfully created and checked out '$TARGET_BRANCH'"
    else
        print_error "Failed to create branch '$TARGET_BRANCH'"
        exit 1
    fi
}

# Function to identify commits to cherry-pick
identify_commits_to_cherry_pick() {
    print_status "Identifying commits to cherry-pick from '$CURRENT_BRANCH'..."
    
    # Get the base commit where dev-e5n diverged from the main sunnypilot branch
    # We'll look for commits that are unique to dev-e5n
    local base_commit
    base_commit=$(git log --oneline "$CURRENT_BRANCH" | tail -1 | cut -d' ' -f1)
    
    # Get commits from dev-e5n that are your changes (after the base sunnypilot commit)
    local commits
    commits=$(git log --reverse --pretty=format:"%H" "$base_commit".."$CURRENT_BRANCH")
    
    if [ -z "$commits" ]; then
        print_warning "No commits found to cherry-pick from '$CURRENT_BRANCH'"
        return 1
    fi
    
    print_status "Commits to cherry-pick:"
    git log --oneline "$base_commit".."$CURRENT_BRANCH"
    
    echo "$commits"
}

# Function to cherry-pick commits with conflict handling
cherry_pick_commits() {
    local commits="$1"
    local commit_count=0
    local success_count=0
    
    for commit in $commits; do
        ((commit_count++))
        local commit_msg
        commit_msg=$(git log --oneline -1 "$commit")
        
        print_status "Cherry-picking commit $commit_count: $commit_msg"
        
        if git cherry-pick "$commit"; then
            ((success_count++))
            print_success "Successfully cherry-picked: $commit_msg"
        else
            print_error "Conflict occurred while cherry-picking: $commit_msg"
            print_status "Please resolve conflicts manually:"
            git status --short
            print_status "After resolving conflicts, run:"
            print_status "  git add ."
            print_status "  git cherry-pick --continue"
            print_status "Or to skip this commit:"
            print_status "  git cherry-pick --skip"
            print_status "Or to abort the cherry-pick:"
            print_status "  git cherry-pick --abort"
            
            # Wait for user input
            read -p "Press Enter after resolving conflicts and continuing cherry-pick, or Ctrl+C to exit..."
            ((success_count++))
        fi
    done
    
    print_success "Cherry-pick completed: $success_count/$commit_count commits processed"
}

# Function to show final status
show_final_status() {
    print_success "Branch update completed!"
    print_status "Current branch: $(git branch --show-current)"
    print_status "Latest commits:"
    git log --oneline -5
    
    print_status "To push the new branch to origin:"
    print_status "  git push -u origin $TARGET_BRANCH"
}

# Main execution function
main() {
    print_status "Starting sunnypilot branch update process..."
    
    # Preliminary checks
    check_git_repo
    check_clean_working_dir
    check_remote
    
    # Fetch updates
    fetch_updates
    
    # Backup existing branch if needed
    backup_existing_branch
    
    # Create new branch
    create_new_branch
    
    # Identify and cherry-pick commits
    local commits
    if commits=$(identify_commits_to_cherry_pick); then
        cherry_pick_commits "$commits"
    else
        print_warning "No commits to cherry-pick, branch created with latest upstream changes only"
    fi
    
    # Show final status
    show_final_status
}

# Handle script interruption
cleanup() {
    print_warning "Script interrupted!"
    print_status "You may need to clean up manually if the process was incomplete"
    exit 130
}

trap cleanup INT

# Help function
show_help() {
    cat << EOF
Sunnypilot Branch Update Script

USAGE:
    $0 [OPTIONS]

DESCRIPTION:
    Fetches the latest dev-c3-new branch from sunnypilot remote,
    creates a local dev-c3-new branch (backing up existing if present),
    and cherry-picks your changes from dev-e5n branch.

OPTIONS:
    -h, --help     Show this help message

REQUIREMENTS:
    - Clean working directory (no uncommitted changes)
    - 'sunnypilot' remote configured
    - Current branch should be 'dev-e5n' or the branch with your changes

EXAMPLES:
    $0                    # Run the full update process
    $0 --help            # Show this help

EOF
}

# Parse command line arguments
case "${1:-}" in
    -h|--help)
        show_help
        exit 0
        ;;
    "")
        main
        ;;
    *)
        print_error "Unknown option: $1"
        show_help
        exit 1
        ;;
esac