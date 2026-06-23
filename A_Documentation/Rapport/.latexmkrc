

system ("mkdir -p build/figures");

@default_files = ('report.tex');
add_cus_dep('glo','gls',0,'makeglossariesrun');
add_cus_dep('acn','acr',0,'makeglossariesrun');

sub makeglossariesrun {
    system("makeglossaries $_[0]");
}

add_cus_dep('glo','gls',0,'makeglo2gls');
sub makeglo2gls {
    system("makeglossaries $_[0]");
}
add_cus_dep('nlo','nls',0,'makenlo2nls');
# Custom dependency and function for nomencl package
add_cus_dep('nlo','nls',0,'makenlo2nls');

sub makenlo2nls {
    my $base = shift;
    my $cmd = "makeindex -s nomencl.ist -o ${base}.nls ${base}.nlo";
    system($cmd);
}

$out_dir = 'build';
$aux_dir = 'build';
$pdflatex = 'xelatex --synctex=1 -interaction=nonstopmode --shell-escape';
$latex = 'latex --synctex=1 -interaction=nonstopmode --shell-escape';

$clean_ext .= " pythontex-files-%R/* pythontex-files-%R";
push @generated_exts, 'pytxcode';

$pythontex = 'pythontex %O %S';
$extra_rule_spec{'pythontex'}  = [ 'internal', '', 'mypythontex', "%Y%R.pytxcode", "%Ypythontex-files-%R/%R.pytxmcr", "%R", 1 ];

sub mypythontex {
   my $result_dir = $aux_dir1."pythontex-files-$$Pbase";
   my $ret = Run_subst( $pythontex, 2 );
   rdb_add_generated( glob "$result_dir/*" );
   open( my $fh, "<", $$Pdest );
   if ($fh) {
      while (<$fh>) {
         if ( /^%PythonTeX dependency:\s+'([^']+)';/ ) {
	     print "Found pythontex dependency '$1'\n";
             rdb_ensure_file( $rule, $aux_dir1.$1 );
	 }
      }
      undef $fh;
   }
   else {
       warn "mypythontex: I could not read '$$Pdest'\n",
            "  to check dependencies\n";
   }
   return $ret;
}