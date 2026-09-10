"""Typed errors raised by repository operations."""

from __future__ import annotations


class RepositoryError(Exception):
    """Base class for repository-layer failures."""


class EntityNotFoundError(RepositoryError):
    """Raised when a required entity does not exist."""


class LeaseOwnershipError(RepositoryError):
    """Raised when a heartbeat or lease mutation is not owned by the caller."""


class InvalidLeaseStateError(RepositoryError):
    """Raised when a lease operation is illegal for the current run status."""


class DuplicateEntityError(RepositoryError):
    """Raised when a unique constraint prevents creating a new entity."""
