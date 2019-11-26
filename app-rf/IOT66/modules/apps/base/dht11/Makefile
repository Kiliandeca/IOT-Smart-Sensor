# Makefile for apps

MODULE = $(shell basename $(shell cd .. && pwd && cd -))
NAME = $(shell basename $(CURDIR))

# Add this to your ~/.vimrc in order to get proper function of :make in vim :
# let $COMPILE_FROM_IDE = 1
ifeq ($(strip $(COMPILE_FROM_IDE)),)
	PRINT_DIRECTORY = --no-print-directory
else
	PRINT_DIRECTORY =
	LANG = C
endif

.PHONY: $(NAME).bin
$(NAME).bin:
	@make -C ../../.. ${PRINT_DIRECTORY} NAME=$(NAME) MODULE=$(MODULE) apps/$(MODULE)/$(NAME)/$@

clean mrproper:
	@make -C ../../.. ${PRINT_DIRECTORY} $@

