$out_dir = 'build';
$pdf_mode = 1;
$bibtex_use = 2;

# Glossaries (acronyms + main glossary)
add_cus_dep('acn', 'acr', 0, 'makeglossaries');
add_cus_dep('glo', 'gls', 0, 'makeglossaries');

# Nomenclature (symbols list)
add_cus_dep('nlo', 'nls', 0, 'makenomenclature');

$clean_ext .= ' acr acn alg glo gls glg nls nlg';

sub makeglossaries {
    my ($base_name, $path) = fileparse($_[0]);
    if ($silent) {
        return system('makeglossaries', '-q', '-d', $path, $base_name);
    }
    return system('makeglossaries', '-d', $path, $base_name);
}

sub makenomenclature {
    return system(
        'makeindex',
        '-s', 'nomencl.ist',
        '-o', "$_[0].nls",
        '-t', "$_[0].nlg",
        "$_[0].nlo"
    );
}
