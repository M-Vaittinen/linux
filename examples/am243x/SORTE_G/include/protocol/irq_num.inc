    .if ! $defined( __irq_num_inc__ )
    .define "1", __irq_num_inc__

IRQ_SVC                 .set    36
IRQ_IN_DATA_READY       .set    35
IRQ_OUT_DATA_READY      .set    34

    .endif
