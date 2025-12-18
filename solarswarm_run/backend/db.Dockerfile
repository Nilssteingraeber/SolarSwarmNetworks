FROM postgres:13

HEALTHCHECK --interval=5s --timeout=5s --retries=5 CMD pg_isready -U postgres || exit 1
