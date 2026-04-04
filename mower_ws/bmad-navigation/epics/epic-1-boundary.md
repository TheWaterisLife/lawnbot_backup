# Epic 1: Boundary Management

> ⚠️ **DEPRECATED**: This epic has been moved to **bmad-boundary**.
> 
> See [bmad-boundary/README.md](../../bmad-boundary/README.md) for the current implementation.

---

## Original Content (for reference)

The original Epic 1 covered boundary recording and management functionality. This has been moved to a separate `boundary_mapper` package for cleaner separation of concerns.

### Migrated Stories

| Story | New Location |
|-------|--------------|
| 1.1 Boundary Recording | [bmad-boundary/stories/story-1.1-recorder-node.md](../../bmad-boundary/stories/story-1.1-recorder-node.md) |
| 1.2 Continuous Recording | [bmad-boundary/stories/story-1.2-continuous-recording.md](../../bmad-boundary/stories/story-1.2-continuous-recording.md) |
| 1.3 Boundary Storage | [bmad-boundary/stories/story-3.2-boundary-manager.md](../../bmad-boundary/stories/story-3.2-boundary-manager.md) |
| 1.4-1.6 | Incorporated into corresponding bmad-boundary stories |

### Why Moved?

- **Separation of Concerns**: Boundary recording is a setup-time activity, navigation is runtime
- **Different Usage Patterns**: User pushes mower during recording, autonomous during navigation
- **Cleaner Dependencies**: Boundary mapping only needs GPS, navigation needs full sensor fusion
