.PHONY: all clean rebuild check

all clean rebuild check:
	@"D:/CCS/ccs/utils/bin/gmake.exe" -C empty -f Makefile.vscode $@

