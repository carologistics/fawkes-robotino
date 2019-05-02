.PHONY: check
check:
check: quickdoc $(if $(subst 0,,$(YAMLLINT)),yamllint) license-check
	$(SILENT)touch $(BASEDIR)/.check_stamp

