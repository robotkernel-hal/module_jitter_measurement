# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = source
BUILDDIR      = build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

install:
	rm -rf $(GHPAGESDIR)
	git clone `git config --get remote.origin.url` --branch gh-pages --depth 1 $(GHPAGESDIR)
	rm -rf $(GHPAGESDIR)/*
	cp -r build/html/* $(GHPAGESDIR)/
	git -C $(GHPAGESDIR) config user.name "Robert Burger"
	git -C $(GHPAGESDIR) config user.email "robert.burger@dlr.de"
	git -C $(GHPAGESDIR) add "*"
	touch $(GHPAGESDIR)/.nojekyll
	git -C $(GHPAGESDIR) add .nojekyll
	-git -C $(GHPAGESDIR) commit -m 'Automatic update of GH pages'
	git -C $(GHPAGESDIR) push

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
