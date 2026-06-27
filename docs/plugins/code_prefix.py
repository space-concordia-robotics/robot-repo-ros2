import logging

from bs4 import BeautifulSoup
from pymdownx.superfences import SuperFencesBlockPreprocessor, SuperFencesCodeExtension

log = logging.getLogger('mkdocs')

SUPPORTED_PREFIXES = {
    'prefix': '*',
    'venv': '(venv)$',
    'dollar': '$',
    'hash': '#',
}


def validator(language, inputs, options, attrs, md):
    """Validate a prefix fence from the list of supported prefixes."""
    options['prefix'] = ''
    for k, v in inputs.items():
        # add all inputs to options
        if k in SUPPORTED_PREFIXES.keys():
            if k == 'prefix':
                # use custom value if custom prefix
                options[k] = v
            else:
                # use pre-defined supported value for key
                options['prefix'] = SUPPORTED_PREFIXES[k]

            if 'lines' not in options:
                options['lines'] = 0  # default to use prefix on all lines if set
        else:
            options[k] = v

    return True


def formatter(source, language, class_name, options, md, **kwargs):
    """Format a given code block with a given prefix if available."""

    prefix = ''
    lines = '0'
    num_lines = len(source.splitlines())
    classes = []
    if 'classes' in kwargs:
        classes = classes + kwargs['classes']
    if class_name != 'prefix':
        classes.append(class_name)

    keep_hl = False

    if options['prefix']:
        prefix = options['prefix']
        # class_name = 'highlight'  # use the prefix as the class name

        if 'lines' not in options or options['lines'] == 0:
            # assume prefix to be set for every line in codeblock
            lines = f"1-{num_lines}"
        else:
            lines = options['lines']
        # handle the case where prefix and hl_lines are specified
        keep_hl = False
        if 'hl_lines' in options:
            keep_hl = True
        if 'hl_lines' not in classes:
            classes.append('hl_lines')
        # set hl_lines to lines to reuse hl_lines formatter logic
        options['hl_lines'] = lines
        # use linenum preprocessor to replace them later with prefix
        if 'linenums' not in options:
            options['linenums'] = '1'

    try:
        sf_ext = SuperFencesCodeExtension()
        config = sf_ext.getConfigs()
        preprocessor = SuperFencesBlockPreprocessor(md)
        preprocessor.config = config
        preprocessor.extension = sf_ext
        preprocessor.get_hl_settings()
        preprocessor.line_count = num_lines
        lines_int = preprocessor.parse_hl_lines(lines)
        soup = BeautifulSoup(
            preprocessor.highlight(
                src=source,
                language=language,
                options=options,
                md=md,
                classes=classes,
                id_value='',
                attrs=None
            ),
            features='lxml'
        )

        if prefix:
            line_nums_col = soup.find("div", {"class": "linenodiv"})
            for index, row in enumerate(line_nums_col.find_all(class_='normal')):
                if (index + 1) in lines_int:  # index starts at 0, lines expected to start at 1
                    row.string = prefix
                else:
                    row.string = ''
                if not keep_hl:
                    # remove highlight if hl_lines was not specified in markdown
                    hl_div = soup.find("span", {"class": "hll"})
                    if hl_div is not None:
                        hl_div.attrs['class'] = ""
    except Exception as err:
        log.error("Caught exception while trying to prefix lines:", exc_info=err)
        raise err
    return soup
